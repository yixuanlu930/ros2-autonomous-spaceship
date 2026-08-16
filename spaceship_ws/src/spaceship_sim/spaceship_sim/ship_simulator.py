#!/usr/bin/env python3
"""
ship_simulator.py — Motor físico 2D de la nave espacial.

Modelo diferencial T invertida:
    - Motor M1 (izquierdo): torque negativo → gira a la derecha
    - Motor M2 (derecho):   torque positivo → gira a la izquierda
    - Ambos iguales:        avance recto
    - Diferencia M1-M2:     avance + giro diferencial

Física:
    F_total = (F1 + F2) * MAX_THRUST   → propulsión en dirección heading
    torque   = (F2 - F1) * ARM_LENGTH  → giro diferencial
    Drag lineal y angular para dar inercia realista.

Viento:
    Fuerza lateral periódica con componente aleatoria.
    La semilla del generador aleatorio se fija en el primer impulso,
    garantizando condiciones idénticas para todos los grupos.
    Parámetros configurables: wind_seed, wind_strength, wind_frequency.

Topics:
    Suscribe: /motor_command  (MotorCommand)
              /clicked_point  (PointStamped) — nuevo target desde RViz
    Publica:  /ship_state     (ShipState)    — 20 Hz
              /ship_target    (PointStamped) — target actual
"""

import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from spaceship_msgs.msg import MotorCommand, ShipState
from spaceship_msgs.srv import SetMotorPower

# ── Constantes físicas ────────────────────────────────────────────────
MAX_THRUST    = 3.0
ARM_LENGTH    = 0.7
LINEAR_DRAG   = 0.5
ANGULAR_DRAG  = 1.2
INERTIA       = 1.5

# ── Condiciones de llegada ────────────────────────────────────────────
ARRIVAL_DIST  = 2.0
ARRIVAL_SPEED = 0.1

# ── Valor centinela "sin target" ──────────────────────────────────────
_NO_TARGET = -999.0


class ShipSimulator(Node):

    def __init__(self):
        super().__init__('ship_simulator')

        # ── Parámetros ────────────────────────────────────────────────
        self.declare_parameter('target_x',       _NO_TARGET)
        self.declare_parameter('target_y',       _NO_TARGET)
        self.declare_parameter('start_x',        0.0)
        self.declare_parameter('start_y',        0.0)
        self.declare_parameter('start_heading',  1.5708)   # π/2 = norte

        # Parámetros de viento
        self.declare_parameter('wind_seed',      42)       # semilla reproducible
        self.declare_parameter('wind_strength',  1.2)      # fuerza máxima (N/kg)
        self.declare_parameter('wind_frequency', 0.15)     # Hz de variación

        tx  = self.get_parameter('target_x').value
        ty  = self.get_parameter('target_y').value
        sx  = self.get_parameter('start_x').value
        sy  = self.get_parameter('start_y').value
        sh  = self.get_parameter('start_heading').value

        self.wind_seed      = self.get_parameter('wind_seed').value
        self.wind_strength  = self.get_parameter('wind_strength').value
        self.wind_frequency = self.get_parameter('wind_frequency').value

        # ── Estado de la nave ─────────────────────────────────────────
        self.x       = sx
        self.y       = sy
        self.heading = sh
        self.vx      = 0.0
        self.vy      = 0.0
        self.omega   = 0.0

        # ── Motores ───────────────────────────────────────────────────
        self.power_m1 = 0.0
        self.power_m2 = 0.0

        # ── Target ────────────────────────────────────────────────────
        self.target_x = tx if tx != _NO_TARGET else None
        self.target_y = ty if ty != _NO_TARGET else None

        # ── Cronómetro ────────────────────────────────────────────────
        self.elapsed_time  = 0.0
        self.timer_running = False
        self.arrived       = False

        # ── Viento ───────────────────────────────────────────────────
        # El RNG se inicializa en el primer impulso (no antes)
        # para garantizar la misma secuencia de viento a todos los grupos
        self._rng: random.Random | None = None   # None hasta primer impulso
        self._wind_time   = 0.0    # tiempo acumulado desde el primer impulso
        self._wind_fx     = 0.0    # fuerza de viento actual en X (mundo)
        self._wind_fy     = 0.0    # fuerza de viento actual en Y (mundo)
        self._wind_target_fx = 0.0 # objetivo de interpolación X
        self._wind_target_fy = 0.0 # objetivo de interpolación Y
        self._next_wind_change = 0.0  # tiempo del próximo cambio de dirección

        # ── Topics ────────────────────────────────────────────────────
        self.create_subscription(
            MotorCommand, '/motor_command', self.on_motor_command, 10)
        self.create_service(
            SetMotorPower, '/set_motor_power', self.on_set_motor_power)
        self.create_subscription(
            PointStamped, '/clicked_point', self.on_clicked_point, 10)

        self.pub_state  = self.create_publisher(ShipState,    '/ship_state',  10)
        self.pub_target = self.create_publisher(PointStamped, '/ship_target', 10)

        # TF broadcaster — publica world→base_link para que RViz tenga el frame "world"
        # y el tool PublishPoint pueda proyectar clicks a coordenadas del mundo
        self._tf_broadcaster = TransformBroadcaster(self)

        # Timer físico a 50 Hz
        self.create_timer(0.02, self.physics_step)
        # Timer de publicación a 20 Hz
        self.create_timer(0.05, self.publish_state)

        self.get_logger().info(
            f'ShipSimulator iniciado.\n'
            f'  Posición inicial : ({self.x:.1f}, {self.y:.1f})\n'
            f'  Target           : '
            f'{f"({self.target_x:.1f}, {self.target_y:.1f})" if self.target_x is not None else "sin target"}\n'
            f'  Viento           : strength={self.wind_strength:.1f}  '
            f'freq={self.wind_frequency:.2f}Hz  seed={self.wind_seed} '
            f'(activa en el primer impulso)'
        )

    # ── Callbacks ─────────────────────────────────────────────────────

    def _apply_motor_power(self, motor_id: int, power_value: int) -> tuple[bool, str]:
        """Aplica una orden de motor validando rango e identificador."""
        if motor_id not in (0, 1, 2):
            return False, f'motor_id inválido: {motor_id}. Usa 0, 1 o 2.'

        power = float(max(0, min(100, int(power_value))))
        if motor_id == 0:
            self.power_m1 = power
            self.power_m2 = power
        elif motor_id == 1:
            self.power_m1 = power
        elif motor_id == 2:
            self.power_m2 = power

        # Primer impulso: arrancar cronómetro Y fijar semilla del viento.
        if power > 0 and not self.timer_running and not self.arrived:
            self.timer_running = True
            self._init_wind()

        return True, f'Motor {motor_id} actualizado a {power:.0f}%'

    def on_motor_command(self, msg: MotorCommand):
        # Se mantiene para pruebas/manual, pero la práctica usa /set_motor_power.
        self._apply_motor_power(int(msg.motor_id), int(msg.power))

    def on_set_motor_power(self, request, response):
        success, message = self._apply_motor_power(
            int(request.motor_id),
            int(request.power)
        )
        response.success = bool(success)
        response.message = message
        return response

    def on_clicked_point(self, msg: PointStamped):
        """Nuevo target desde RViz — resetea cronómetro y viento."""
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        self._reset_run()
        self.get_logger().info(
            f'🎯 Nuevo target: ({self.target_x:.2f}, {self.target_y:.2f})'
        )
        self._publish_target()

    # ── Inicialización y reset ────────────────────────────────────────

    def _init_wind(self):
        """
        Inicializa el generador de viento con la semilla configurada.
        Llamado exactamente una vez: en el primer impulso de motor.
        Esto garantiza que todos los grupos experimentan la misma
        secuencia de viento independientemente de cuándo lancen el nodo.
        """
        self._rng = random.Random(self.wind_seed)
        self._wind_time = 0.0
        self._wind_fx   = 0.0
        self._wind_fy   = 0.0
        self._next_wind_change = 0.0   # primer cambio inmediato
        self.get_logger().info(
            f'💨 Viento inicializado (seed={self.wind_seed})'
        )

    def _reset_run(self):
        """Resetea cronómetro, viento y estado de llegada."""
        self.elapsed_time  = 0.0
        self.timer_running = False
        self.arrived       = False
        self._rng          = None
        self._wind_time    = 0.0
        self._wind_fx      = 0.0
        self._wind_fy      = 0.0

    # ── Viento ────────────────────────────────────────────────────────

    def _update_wind(self, dt: float):
        """
        Actualiza la fuerza de viento con interpolación suave.

        El viento cambia de dirección cada ~1/wind_frequency segundos,
        interpolando suavemente entre direcciones aleatorias.
        La secuencia es completamente determinista dado wind_seed.
        """
        if self._rng is None:
            return   # viento inactivo hasta primer impulso

        self._wind_time += dt

        # Generar nuevo objetivo de viento cuando toca
        if self._wind_time >= self._next_wind_change:
            angle    = self._rng.uniform(0, 2 * math.pi)
            strength = self._rng.uniform(
                self.wind_strength * 0.3,
                self.wind_strength
            )
            self._wind_target_fx = strength * math.cos(angle)
            self._wind_target_fy = strength * math.sin(angle)
            # Próximo cambio: periodo base + variación aleatoria ±30%
            period = 1.0 / max(self.wind_frequency, 0.01)
            self._next_wind_change = (
                self._wind_time + period * self._rng.uniform(0.7, 1.3)
            )

        # Interpolación suave (low-pass) hacia el objetivo
        alpha = min(1.0, dt * 2.0)   # constante de tiempo ~0.5s
        self._wind_fx += (self._wind_target_fx - self._wind_fx) * alpha
        self._wind_fy += (self._wind_target_fy - self._wind_fy) * alpha

    # ── Física ────────────────────────────────────────────────────────

    def physics_step(self):
        dt = 0.02

        f1 = (self.power_m1 / 100.0) * MAX_THRUST
        f2 = (self.power_m2 / 100.0) * MAX_THRUST

        # Traslación: solo la componente común (promedio)
        ft = (f1 + f2) / 2.0  # ← dividir entre 2, no sumar
        ax = ft * math.cos(self.heading) - self.vx * LINEAR_DRAG
        ay = ft * math.sin(self.heading) - self.vy * LINEAR_DRAG

        # Rotación: diferencial puro
        torque = (f1 - f2) * ARM_LENGTH   # M1 izq (Y−) → empuja → gira derecha ✓
        alpha = torque / INERTIA - self.omega * ANGULAR_DRAG

        # Viento (solo activo desde el primer impulso)
        self._update_wind(dt)
        ax += self._wind_fx
        ay += self._wind_fy

        # Integración Euler
        self.vx      += ax * dt
        self.vy      += ay * dt
        self.omega   += alpha * dt
        self.x       += self.vx * dt
        self.y       += self.vy * dt
        self.heading += self.omega * dt

        # Normalizar heading
        while self.heading >  math.pi: self.heading -= 2 * math.pi
        while self.heading < -math.pi: self.heading += 2 * math.pi

        # Cronómetro
        if self.timer_running and not self.arrived:
            self.elapsed_time += dt

        # Detección de llegada
        if self.target_x is not None and not self.arrived:
            dx    = self.target_x - self.x
            dy    = self.target_y - self.y
            dist  = math.sqrt(dx*dx + dy*dy)
            speed = math.sqrt(self.vx**2 + self.vy**2)
            if dist < ARRIVAL_DIST and speed < ARRIVAL_SPEED:
                self.arrived       = True
                self.timer_running = False
                self.get_logger().info(
                    f'✅ LLEGADO en {self.elapsed_time:.2f}s  '
                    f'dist={dist:.2f}m  speed={speed:.3f}m/s'
                )

    # ── Publicación ───────────────────────────────────────────────────

    def publish_state(self):
        now = self.get_clock().now().to_msg()

        # ── TF: world → base_link ─────────────────────────────────────
        # Necesario para que RViz reconozca el frame "world" y el tool
        # PublishPoint pueda transformar clicks a coordenadas del mundo.
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'world'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.heading / 2.0)
        t.transform.rotation.w = math.cos(self.heading / 2.0)
        self._tf_broadcaster.sendTransform(t)

        # ── ShipState ─────────────────────────────────────────────────
        msg = ShipState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.x            = self.x
        msg.y            = self.y
        msg.heading      = self.heading
        msg.vx           = self.vx
        msg.vy           = self.vy
        msg.omega        = self.omega
        msg.power_m1     = float(self.power_m1)
        msg.power_m2     = float(self.power_m2)
        msg.elapsed_time = self.elapsed_time
        msg.arrived      = self.arrived
        msg.target_x     = self.target_x if self.target_x is not None else 0.0
        msg.target_y     = self.target_y if self.target_y is not None else 0.0
        self.pub_state.publish(msg)

    def _publish_target(self):
        if self.target_x is None:
            return
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.point.x = self.target_x
        msg.point.y = self.target_y
        self.pub_target.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ShipSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
