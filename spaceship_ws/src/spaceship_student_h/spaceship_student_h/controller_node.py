#!/usr/bin/env python3
"""Controlador autónomo para spaceship_sim.

Características principales:
- Suscripción a /ship_state.
- Control de motores mediante el servicio asíncrono /set_motor_power.
- Ley de control vectorial PD: aceleración deseada = kp_pos * error_pos - kd_vel * velocidad.
- Fases ORIENT / THRUST / BRAKE / SETTLE / ARRIVED.
- Versión estable con guiado vertical suave: apunta ligeramente por encima del target
  durante la aproximación para no quedarse por debajo de la cruz.
- Publicación de ControlDebug.msg y marcadores RViz.
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from spaceship_msgs.msg import ShipState
from spaceship_msgs.srv import SetMotorPower
from spaceship_msgs.msg import ControlDebug


ARRIVAL_DIST = 2.0
ARRIVAL_SPEED = 0.1
MAX_THRUST = 3.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def make_color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


def make_point(x: float, y: float, z: float = 0.0) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


class SpaceshipController(Node):
    """Nodo controlador del alumno."""

    def __init__(self):
        super().__init__('spaceship_controller')

        # Target por defecto. Si el simulador publica un target válido en ShipState,
        # se usa el target del simulador; si no, se usa este valor.
        self.declare_parameter('target_x', 10.0)
        self.declare_parameter('target_y', 8.0)

        # Parámetros de control configurables desde launch.
        self.declare_parameter('kp_pos', 0.8)
        self.declare_parameter('kd_vel', 2.2)
        self.declare_parameter('max_power', 100.0)
        self.declare_parameter('min_thrust_power', 12.0)
        self.declare_parameter('turn_gain', 42.0)
        self.declare_parameter('omega_damping', 18.0)
        self.declare_parameter('angle_tolerance', 0.18)
        self.declare_parameter('hard_angle_tolerance', 0.85)
        self.declare_parameter('service_period', 0.08)
        self.declare_parameter('turn_sign', 1.0)
        self.declare_parameter('use_clicked_point', True)

        # Parámetros mantenidos por compatibilidad con versiones/launch anteriores.
        self.declare_parameter('near_distance', 3.2)
        self.declare_parameter('final_stop_distance', 0.35)
        self.declare_parameter('final_stop_speed', 0.05)
        self.declare_parameter('fine_power', 16.0)
        self.declare_parameter('fine_turn_gain', 28.0)
        self.declare_parameter('fine_speed_gain', 18.0)

        # Mejora segura sobre la versión estable:
        # durante la aproximación se apunta un poco por encima del target real.
        self.declare_parameter('vertical_guidance_offset', 1.15)
        self.declare_parameter('guidance_release_distance', 4.0)

        self.default_target_x = float(self.get_parameter('target_x').value)
        self.default_target_y = float(self.get_parameter('target_y').value)

        self.kp_pos = float(self.get_parameter('kp_pos').value)
        self.kd_vel = float(self.get_parameter('kd_vel').value)
        self.max_power = float(self.get_parameter('max_power').value)
        self.min_thrust_power = float(self.get_parameter('min_thrust_power').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.omega_damping = float(self.get_parameter('omega_damping').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)
        self.hard_angle_tolerance = float(self.get_parameter('hard_angle_tolerance').value)
        self.service_period = float(self.get_parameter('service_period').value)
        self.turn_sign = float(self.get_parameter('turn_sign').value)
        self.use_clicked_point = bool(self.get_parameter('use_clicked_point').value)

        self.near_distance = float(self.get_parameter('near_distance').value)
        self.final_stop_distance = float(self.get_parameter('final_stop_distance').value)
        self.final_stop_speed = float(self.get_parameter('final_stop_speed').value)
        self.fine_power = float(self.get_parameter('fine_power').value)
        self.fine_turn_gain = float(self.get_parameter('fine_turn_gain').value)
        self.fine_speed_gain = float(self.get_parameter('fine_speed_gain').value)

        self.vertical_guidance_offset = float(self.get_parameter('vertical_guidance_offset').value)
        self.guidance_release_distance = float(self.get_parameter('guidance_release_distance').value)

        self.target_x: Optional[float] = self.default_target_x
        self.target_y: Optional[float] = self.default_target_y

        self.client = self.create_client(SetMotorPower, '/set_motor_power')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /set_motor_power...')

        self.state_sub = self.create_subscription(
            ShipState,
            '/ship_state',
            self.on_ship_state,
            10,
        )

        if self.use_clicked_point:
            self.click_sub = self.create_subscription(
                PointStamped,
                '/clicked_point',
                self.on_clicked_point,
                10,
            )

        self.debug_pub = self.create_publisher(
            ControlDebug,
            '/control_debug',
            10,
        )

        # Publicamos en el mismo topic de RViz del simulador para que se vea con la config dada.
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10,
        )

        self.last_m1: Optional[int] = None
        self.last_m2: Optional[int] = None
        self.last_service_time = self.get_clock().now()

        # Estimación de viento (filtro paso-bajo)
        self._wind_est_x = 0.0
        self._wind_est_y = 0.0
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_heading = 0.0
        self._last_power_m1 = 0.0
        self._last_power_m2 = 0.0

        self.trail: list[Tuple[float, float]] = []
        self.max_trail_points = 800

        self.get_logger().info(
            'Controlador iniciado: target=(%.2f, %.2f), kp_pos=%.2f, kd_vel=%.2f, max_power=%.0f'
            % (
                self.default_target_x,
                self.default_target_y,
                self.kp_pos,
                self.kd_vel,
                self.max_power,
            )
        )

    def on_clicked_point(self, msg: PointStamped) -> None:
        self.target_x = float(msg.point.x)
        self.target_y = float(msg.point.y)
        self.trail.clear()
        self.get_logger().info(
            'Nuevo target recibido por click: (%.2f, %.2f)'
            % (self.target_x, self.target_y)
        )

    def _current_target(self, msg: ShipState) -> Tuple[float, float]:
        """Obtiene el target actual desde ShipState o desde parámetros."""

        sim_target_valid = True

        if not math.isfinite(msg.target_x) or not math.isfinite(msg.target_y):
            sim_target_valid = False

        # En el simulador, -999 suele indicar que no hay target fijado.
        if abs(msg.target_x + 999.0) < 1e-6 and abs(msg.target_y + 999.0) < 1e-6:
            sim_target_valid = False

        # Si el simulador publica un target razonable, lo seguimos.
        if sim_target_valid and (abs(msg.target_x) > 1e-9 or abs(msg.target_y) > 1e-9):
            self.target_x = float(msg.target_x)
            self.target_y = float(msg.target_y)

        if self.target_x is None or self.target_y is None:
            self.target_x = self.default_target_x
            self.target_y = self.default_target_y

        return self.target_x, self.target_y

    def _send_motor(self, motor_id: int, power: int) -> None:
        req = SetMotorPower.Request()
        req.motor_id = int(motor_id)
        req.power = int(clamp(power, 0, 100))

        future = self.client.call_async(req)
        future.add_done_callback(self._service_done_callback)

    def _service_done_callback(self, future) -> None:
        try:
            response = future.result()
            if response is not None and not response.success:
                self.get_logger().warn(response.message)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Fallo llamando a /set_motor_power: {exc}')

    def _send_motors(self, m1: float, m2: float, force: bool = False) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_service_time).nanoseconds * 1e-9

        if not force and dt < self.service_period:
            return

        m1_i = int(round(clamp(m1, 0, 100)))
        m2_i = int(round(clamp(m2, 0, 100)))

        if force or self.last_m1 != m1_i:
            self._send_motor(1, m1_i)
            self.last_m1 = m1_i

        if force or self.last_m2 != m2_i:
            self._send_motor(2, m2_i)
            self.last_m2 = m2_i

        self.last_service_time = now

    def _guidance_target(self, tx: float, ty: float, dist_real: float) -> Tuple[float, float]:
        """Devuelve un target virtual ligeramente elevado durante la aproximación.

        La idea es evitar que la nave llegue visualmente por debajo de la cruz.
        Lejos del objetivo se apunta un poco por encima; al acercarse se reduce
        gradualmente el offset hasta apuntar al target real.
        """

        release_distance = max(self.guidance_release_distance, ARRIVAL_DIST + 0.1)

        if dist_real > release_distance:
            offset = self.vertical_guidance_offset
        elif dist_real > ARRIVAL_DIST:
            alpha = (dist_real - ARRIVAL_DIST) / (release_distance - ARRIVAL_DIST)
            alpha = clamp(alpha, 0.0, 1.0)
            offset = self.vertical_guidance_offset * alpha
        else:
            offset = 0.0

        return tx, ty + offset

    def on_ship_state(self, msg: ShipState) -> None:
        # Estimar viento por residuo de aceleración
        dt = 0.05
        thrust = ((self._last_power_m1 + self._last_power_m2) / 2.0 / 100.0) * 3.0
        ax_model = thrust * math.cos(self._last_heading) - 0.5 * self._last_vx
        ay_model = thrust * math.sin(self._last_heading) - 0.5 * self._last_vy
        ax_real = (msg.vx - self._last_vx) / dt
        ay_real = (msg.vy - self._last_vy) / dt
        alpha = 0.15
        self._wind_est_x += alpha * ((ax_real - ax_model) - self._wind_est_x)
        self._wind_est_y += alpha * ((ay_real - ay_model) - self._wind_est_y)
        self._last_vx = msg.vx
        self._last_vy = msg.vy
        self._last_heading = msg.heading
        self._last_power_m1 = msg.power_m1
        self._last_power_m2 = msg.power_m2

        tx, ty = self._current_target(msg)

        # Distancia real al target oficial.
        dx_real = tx - msg.x
        dy_real = ty - msg.y
        dist = math.hypot(dx_real, dy_real)
        speed = math.hypot(msg.vx, msg.vy)

        # Target virtual usado por el controlador.
        guidance_tx, guidance_ty = self._guidance_target(tx, ty, dist)

        dx = guidance_tx - msg.x
        dy = guidance_ty - msg.y
        guidance_dist = math.hypot(dx, dy)

        # Ley de control vectorial PD sobre el target virtual.
        desired_ax = self.kp_pos * dx - self.kd_vel * msg.vx - self._wind_est_x
        desired_ay = self.kp_pos * dy - self.kd_vel * msg.vy - self._wind_est_y
        desired_norm = math.hypot(desired_ax, desired_ay)

        if desired_norm < 1e-6:
            desired_heading = msg.heading
        else:
            desired_heading = math.atan2(desired_ay, desired_ax)

        angle_error = normalize_angle(desired_heading - msg.heading)

        if msg.arrived or (dist < ARRIVAL_DIST and speed < ARRIVAL_SPEED):
            phase = 'ARRIVED'
            m1 = 0.0
            m2 = 0.0
            self._send_motors(m1, m2, force=True)

        elif dist < ARRIVAL_DIST:
            phase = 'SETTLE'
            m1 = 0.0
            m2 = 0.0
            self._send_motors(m1, m2, force=True)

        else:
            closing_speed = 0.0
            if guidance_dist > 1e-6:
                closing_speed = (msg.vx * dx + msg.vy * dy) / guidance_dist

            if abs(angle_error) > self.hard_angle_tolerance:
                phase = 'ORIENT'
                base_power = 0.0
            else:
                alignment = max(0.0, math.cos(angle_error))
                desired_power = (desired_norm / MAX_THRUST) * 100.0 * alignment
                base_power = clamp(desired_power, 0.0, self.max_power)

                if base_power > 0.0:
                    base_power = max(base_power, self.min_thrust_power)

                # Versión estable: frenar al acercarse.
                if dist < 4.0 or (dist < 8.0 and closing_speed > 0.7):
                    phase = 'BRAKE'
                elif speed < 0.05 and dist < 2.8:
                    phase = 'HOVER'
                else:
                    phase = 'THRUST'

            differential = self.turn_sign * (
                self.turn_gain * angle_error - self.omega_damping * msg.omega
            )

            m1 = base_power + differential
            m2 = base_power - differential

            if phase == 'ORIENT':
                m1 = clamp(m1, 0.0, self.max_power)
                m2 = clamp(m2, 0.0, self.max_power)

                if max(m1, m2) < self.min_thrust_power:
                    if differential >= 0.0:
                        m1 = self.min_thrust_power
                        m2 = 0.0
                    else:
                        m1 = 0.0
                        m2 = self.min_thrust_power
            else:
                m1 = clamp(m1, 0.0, self.max_power)
                m2 = clamp(m2, 0.0, self.max_power)

            self._send_motors(m1, m2)

        self._update_trail(msg.x, msg.y)

        self._publish_debug(
            msg,
            phase,
            dist,
            angle_error,
            speed,
            desired_heading,
            desired_ax,
            desired_ay,
            m1,
            m2,
        )

        self._publish_markers(
            msg,
            phase,
            dist,
            speed,
            desired_heading,
            tx,
            ty,
            m1,
            m2,
        )

    def _update_trail(self, x: float, y: float) -> None:
        self.trail.append((float(x), float(y)))

        if len(self.trail) > self.max_trail_points:
            self.trail.pop(0)

    def _publish_debug(
        self,
        msg: ShipState,
        phase: str,
        dist: float,
        angle_error: float,
        speed: float,
        desired_heading: float,
        desired_ax: float,
        desired_ay: float,
        m1: float,
        m2: float,
    ) -> None:
        debug = ControlDebug()
        debug.header = msg.header
        debug.phase = phase
        debug.distance_to_target = float(dist)
        debug.angle_error = float(angle_error)
        debug.speed = float(speed)
        debug.desired_heading = float(desired_heading)
        debug.desired_accel_x = float(desired_ax)
        debug.desired_accel_y = float(desired_ay)
        debug.power_m1 = float(clamp(m1, 0, 100))
        debug.power_m2 = float(clamp(m2, 0, 100))

        self.debug_pub.publish(debug)

    def _base_marker(
        self,
        marker_id: int,
        marker_type: int,
        now,
        ns: str = 'student_controller',
    ) -> Marker:
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = now
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _publish_markers(
        self,
        msg: ShipState,
        phase: str,
        dist: float,
        speed: float,
        desired_heading: float,
        tx: float,
        ty: float,
        m1: float,
        m2: float,
    ) -> None:
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        # 1) Texto de fase, distancia, velocidad y tiempo.
        text = self._base_marker(100, Marker.TEXT_VIEW_FACING, now)
        text.pose.position = make_point(msg.x, msg.y + 2.8, 0.8)
        text.scale.z = 0.45
        text.color = make_color(1.0, 1.0, 1.0, 1.0)
        text.text = (
            f'{phase}\n'
            f'd={dist:.2f} m  v={speed:.2f} m/s\n'
            f't={msg.elapsed_time:.2f} s\n'
            f'M1={int(clamp(m1, 0, 100))}% M2={int(clamp(m2, 0, 100))}%'
        )
        arr.markers.append(text)

        # 2) Flecha de heading deseado.
        desired = self._base_marker(101, Marker.ARROW, now)
        desired.scale.x = 0.08
        desired.scale.y = 0.18
        desired.scale.z = 0.18
        desired.color = make_color(0.1, 1.0, 0.1, 0.9)
        desired.points = [
            make_point(msg.x, msg.y, 0.25),
            make_point(
                msg.x + 3.0 * math.cos(desired_heading),
                msg.y + 3.0 * math.sin(desired_heading),
                0.25,
            ),
        ]
        arr.markers.append(desired)

        # 3) Línea de error hasta el objetivo real.
        error_line = self._base_marker(102, Marker.LINE_STRIP, now)
        error_line.scale.x = 0.05
        error_line.color = make_color(1.0, 0.75, 0.1, 0.9)
        error_line.points = [
            make_point(msg.x, msg.y, 0.15),
            make_point(tx, ty, 0.15),
        ]
        arr.markers.append(error_line)

        # 4) Barra de distancia restante.
        bar = self._base_marker(103, Marker.CUBE, now)
        bar.pose.position = make_point(tx, ty - 3.0, 0.15)
        bar.scale.x = clamp(dist / 4.0, 0.05, 5.0)
        bar.scale.y = 0.15
        bar.scale.z = 0.15
        bar.color = make_color(0.2, 0.6, 1.0, 0.75)
        arr.markers.append(bar)

        # 5) Traza del controlador.
        trail = self._base_marker(104, Marker.LINE_STRIP, now, ns='student_trail')
        trail.scale.x = 0.035
        trail.color = make_color(1.0, 1.0, 1.0, 0.35)

        for x, y in self.trail:
            trail.points.append(make_point(x, y, 0.05))

        arr.markers.append(trail)

        self.marker_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpaceshipController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._send_motors(0, 0, force=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()