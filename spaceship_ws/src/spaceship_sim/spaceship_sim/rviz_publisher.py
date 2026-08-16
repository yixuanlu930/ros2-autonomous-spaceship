#!/usr/bin/env python3
"""
rviz_publisher.py — Publica marcadores de visualización para RViz.

Marcadores publicados en /visualization_marker_array:
  ns="ship"       id=0  → cuerpo de la nave (T invertida, TRIANGLE_LIST)
  ns="flames"     id=1  → llama motor M1 izquierdo (naranja)
  ns="flames"     id=2  → llama motor M2 derecho (morado)
  ns="hud"        id=0  → cronómetro (TEXT_VIEW_FACING)
  ns="hud"        id=1  → barra potencia M1 (CUBE)
  ns="hud"        id=2  → barra potencia M2 (CUBE)
  ns="trail"      id=0  → trayectoria (LINE_STRIP)
  ns="target"     id=0  → zona de llegada (CYLINDER)
  ns="target"     id=1  → cruz del target (LINE_LIST)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from spaceship_msgs.msg import ShipState

_NO_TARGET = -999.0
MAX_TRAIL  = 500   # máximo de puntos en la trayectoria


def _header(stamp, frame='world'):
    h = Header()
    h.stamp    = stamp
    h.frame_id = frame
    return h


def _color(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _point(x, y, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def _quaternion_z(heading):
    """Quaternion de rotación alrededor de Z."""
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(heading / 2.0)
    q.w = math.cos(heading / 2.0)
    return q


class RvizPublisher(Node):

    def __init__(self):
        super().__init__('rviz_publisher')

        self.target_x: float | None = None
        self.target_y: float | None = None
        self.trail: list[tuple[float, float]] = []

        self.sub_state = self.create_subscription(
            ShipState, '/ship_state', self.on_ship_state, 10)

        self.sub_target = self.create_subscription(
            PointStamped, '/ship_target', self.on_ship_target, 10)

        # También escucha /clicked_point para actualizar el target visualmente
        self.sub_click = self.create_subscription(
            PointStamped, '/clicked_point', self.on_clicked_point, 10)

        self.pub_markers = self.create_publisher(
            MarkerArray, '/visualization_marker_array', 10)

        self.get_logger().info('RvizPublisher iniciado.')

    # ── Callbacks ─────────────────────────────────────────────────────

    def on_ship_state(self, msg: ShipState):
        # Actualizar target desde el estado si aún no lo tenemos
        if self.target_x is None and msg.target_x != 0.0:
            self.target_x = msg.target_x
            self.target_y = msg.target_y

        # Acumular trayectoria
        self.trail.append((msg.x, msg.y))
        if len(self.trail) > MAX_TRAIL:
            self.trail.pop(0)

        self._publish(msg)

    def on_ship_target(self, msg: PointStamped):
        self.target_x = msg.point.x
        self.target_y = msg.point.y

    def on_clicked_point(self, msg: PointStamped):
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        self.trail.clear()   # reset trayectoria al cambiar target

    # ── Publicación ───────────────────────────────────────────────────

    def _publish(self, s: ShipState):
        now   = self.get_clock().now().to_msg()
        marks = MarkerArray()

        marks.markers.append(self._ship_body(s, now))
        marks.markers.append(self._flame(s, now, motor=1))
        marks.markers.append(self._flame(s, now, motor=2))
        marks.markers.append(self._timer_text(s, now))
        #marks.markers.append(self._power_bar(s, now, motor=1))
        #marks.markers.append(self._power_bar(s, now, motor=2))
        marks.markers.append(self._trail(now))

        if self.target_x is not None:
            marks.markers.append(self._target_zone(now))
            marks.markers.append(self._target_cross(now))

        self.pub_markers.publish(marks)

    # ── Marcadores ────────────────────────────────────────────────────

    def _ship_body(self, s: ShipState, now) -> Marker:
        """Cuerpo de la nave: T invertida con nariz apuntando a +X local."""
        m = Marker()
        m.header = _header(now)
        m.ns, m.id = 'ship', 0
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD
        m.pose.position = _point(s.x, s.y)
        m.pose.orientation = _quaternion_z(s.heading)
        m.scale.x = m.scale.y = m.scale.z = 1.0

        if s.arrived:
            m.color = _color(0.1, 1.0, 0.2)
        else:
            m.color = _color(0.0, 0.9, 1.0)

        # Fuselaje — nariz en +X local
        fuselage = [
            (0.8, 0.0),  # nariz
            (-0.2, -0.15),  # base inf
            (-0.2, 0.15),  # base sup
        ]
        # Barra horizontal de la T (motores en ±Y local)
        t_bar = [
            (-0.1, -0.7), (-0.1, 0.7), (-0.3, -0.7),
            (-0.1, 0.7), (-0.3, 0.7), (-0.3, -0.7),
        ]
        for px, py in fuselage + t_bar:
            m.points.append(_point(px, py))
            m.colors.append(m.color)

        return m

    def _flame(self, s: ShipState, now, motor: int) -> Marker:
        """
        Llama del motor motor=1 (Y−, naranja) o motor=2 (Y+, morado).
        Escala proporcional a la potencia.
        Si potencia = 0, manda DELETE.
        """
        power = s.power_m1 if motor == 1 else s.power_m2
        anchor_y = -0.7 if motor == 1 else 0.7  # eje Y local (barra de la T)

        m = Marker()
        m.header = _header(now)
        m.ns = 'flames'
        m.id = motor
        m.action = Marker.ADD

        if power < 1.0:
            m.action = Marker.DELETE
            return m

        flame_len = (power / 100.0) * 0.8  # 0 – 0.8 metros

        m.type = Marker.TRIANGLE_LIST
        m.pose.position = _point(s.x, s.y)
        m.pose.orientation = _quaternion_z(s.heading)
        m.scale.x = m.scale.y = m.scale.z = 1.0

        if motor == 1:
            m.color = _color(1.0, 0.55, 0.0, 0.85)  # naranja
        else:
            m.color = _color(0.7, 0.2, 1.0, 0.85)  # morado

        # Triángulo: base en el motor, punta hacia −X local (cola de la nave)
        flame_pts = [
            (-0.2, anchor_y - 0.1),
            (-0.2, anchor_y + 0.1),
            (-0.2 - flame_len, anchor_y),
        ]
        for px, py in flame_pts:
            m.points.append(_point(px, py))
            m.colors.append(m.color)

        return m

    def _timer_text(self, s: ShipState, now) -> Marker:
        """Cronómetro flotante sobre la nave (TEXT_VIEW_FACING)."""
        m = Marker()
        m.header          = _header(now)
        m.ns, m.id        = 'hud', 0
        m.type            = Marker.TEXT_VIEW_FACING
        m.action          = Marker.ADD
        m.pose.position   = _point(s.x, s.y + 1.8)
        m.pose.orientation.w = 1.0
        m.scale.z         = 0.45  # altura del texto en metros

        if s.arrived:
            m.text  = f'ARRIVED  {s.elapsed_time:.2f}s'
            m.color = _color(0.1, 1.0, 0.2)
        elif s.elapsed_time > 0.0:
            m.text  = f'{s.elapsed_time:.2f}s'
            m.color = _color(1.0, 0.9, 0.0)
        else:
            m.text  = 'READY'
            m.color = _color(0.6, 0.6, 0.6)

        return m

    def _power_bar(self, s: ShipState, now, motor: int) -> Marker:
        """Barra de potencia debajo de la nave (CUBE escalado)."""
        power = s.power_m1 if motor == 1 else s.power_m2
        offset_x = -0.7 if motor == 1 else 0.7
        col = _color(1.0, 0.55, 0.0) if motor == 1 else _color(0.7, 0.2, 1.0)

        m = Marker()
        m.header   = _header(now)
        m.ns, m.id = 'hud', motor
        m.type     = Marker.CUBE
        m.action   = Marker.ADD

        bar_len = max(0.01, (power / 100.0) * 1.2)
        # Centro de la barra en coordenadas mundo (debajo de la nave)
        cx = s.x + offset_x * math.cos(s.heading) - (-1.2) * math.sin(s.heading)
        cy = s.y + offset_x * math.sin(s.heading) + (-1.2) * math.cos(s.heading)

        m.pose.position    = _point(cx, cy, 0.0)
        m.pose.orientation = _quaternion_z(s.heading)
        m.scale.x = 0.12
        m.scale.y = bar_len
        m.scale.z = 0.08
        m.color   = col
        return m

    def _trail(self, now) -> Marker:
        """Trayectoria recorrida (LINE_STRIP)."""
        m = Marker()
        m.header   = _header(now)
        m.ns, m.id = 'trail', 0
        m.type     = Marker.LINE_STRIP
        m.action   = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x  = 0.04
        m.color    = _color(0.3, 0.6, 1.0, 0.5)

        for x, y in self.trail:
            m.points.append(_point(x, y))
            m.colors.append(m.color)

        return m

    def _target_zone(self, now) -> Marker:
        """Cilindro semitransparente en el target (radio 2 m)."""
        m = Marker()
        m.header   = _header(now)
        m.ns, m.id = 'target', 0
        m.type     = Marker.CYLINDER
        m.action   = Marker.ADD
        m.pose.position    = _point(self.target_x, self.target_y, -0.05)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 4.0   # diámetro = 2*radio = 4 m
        m.scale.z = 0.05
        m.color   = _color(0.1, 1.0, 0.2, 0.25)
        return m

    def _target_cross(self, now) -> Marker:
        """Cruz en el centro del target (LINE_LIST)."""
        m = Marker()
        m.header   = _header(now)
        m.ns, m.id = 'target', 1
        m.type     = Marker.LINE_LIST
        m.action   = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x  = 0.06
        m.color    = _color(0.1, 1.0, 0.2, 0.9)
        tx, ty     = self.target_x, self.target_y
        arm        = 0.5
        pts = [
            (tx - arm, ty), (tx + arm, ty),
            (tx, ty - arm), (tx, ty + arm),
        ]
        for px, py in pts:
            m.points.append(_point(px, py))
        return m


def main(args=None):
    rclpy.init(args=args)
    node = RvizPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
# NOTA: el viento ya es visible indirectamente en la trayectoria.
# Si se quiere añadir un marcador de flecha de viento, añadir aquí
# un Marker de tipo ARROW suscrito a un topic /wind_state (opcional).
