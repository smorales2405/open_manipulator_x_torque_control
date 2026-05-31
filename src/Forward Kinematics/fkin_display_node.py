#!/usr/bin/env python3
"""
fkin_display_node — OpenManipulator-X Forward Kinematics Display
================================================================
Suscribe /joint_states, calcula la cinematica directa analitica
(identica a open_manx_fkin.m) e imprime x, y, z, phi en terminal.

Publica:
  · TF estatico  "target_yf"   — frame en la posicion objetivo yf
  · /fkin_markers (MarkerArray) — ejes RGB en end_effector_link y en target_yf

Parametros ROS (ver config/fkin_params.yaml):
  yf.x, yf.y, yf.z   [m]   posicion objetivo del efector final
  yf.phi              [rad] angulo de pitch objetivo (solo informativo)
  print_rate_hz       [Hz]  frecuencia de impresion en terminal
  axis_length         [m]   longitud de las flechas de los ejes
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import StaticTransformBroadcaster


# ─────────────────────────────────────────────────────────────────────────────
#  Cinematica directa — parametros del URDF openmani.urdf
#  Identica a open_manx_fkin.m
# ─────────────────────────────────────────────────────────────────────────────
_X_BASE = 0.012          # joint1 origin x  [m]
_Z_BASE = 0.017 + 0.0595 # joint1 z + joint2 z  [m]
_X23    = 0.024          # joint3 origin x (en frame joint2)  [m]
_Z23    = 0.128          # joint3 origin z (en frame joint2)  [m]
_L34    = 0.124          # joint4 origin x (en frame joint3)  [m]
_L4E    = 0.126          # end_effector origin x (en frame joint4)  [m]


def open_manx_fkin(q1: float, q2: float, q3: float, q4: float):
    """
    Retorna (x, y, z, phi) del end_effector_link en el frame world.
    phi = q2+q3+q4  (angulo de pitch del gripper respecto a la horizontal).
    """
    r = (_X23 * math.cos(q2) + _Z23 * math.sin(q2)
         + _L34 * math.cos(q2 + q3)
         + _L4E * math.cos(q2 + q3 + q4))
    z = (_Z_BASE
         + (-_X23 * math.sin(q2) + _Z23 * math.cos(q2))
         - _L34 * math.sin(q2 + q3)
         - _L4E * math.sin(q2 + q3 + q4))
    x   = _X_BASE + r * math.cos(q1)
    y   = r * math.sin(q1)
    phi = q2 + q3 + q4
    return x, y, z, phi


# ─────────────────────────────────────────────────────────────────────────────
#  Nodo principal
# ─────────────────────────────────────────────────────────────────────────────
class FkinDisplayNode(Node):

    _JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']

    def __init__(self):
        super().__init__('fkin_display')

        # ── Declarar parametros ──────────────────────────────────────────
        self.declare_parameter('yf.x',          0.2)
        self.declare_parameter('yf.y',         -0.13)
        self.declare_parameter('yf.z',          0.2)
        self.declare_parameter('yf.phi',        0.0)
        self.declare_parameter('print_rate_hz', 2.0)
        self.declare_parameter('axis_length',   0.08)

        yf_x   = self.get_parameter('yf.x').value
        yf_y   = self.get_parameter('yf.y').value
        yf_z   = self.get_parameter('yf.z').value
        yf_phi = self.get_parameter('yf.phi').value
        rate   = float(self.get_parameter('print_rate_hz').value)
        self._L = float(self.get_parameter('axis_length').value)

        self._yf = (yf_x, yf_y, yf_z, yf_phi)
        self._q  = [0.0, 0.0, 0.0, 0.0]   # [q1, q2, q3, q4]
        self._js_idx: dict[str, int] = {}  # nombre → indice en JointState

        # ── Suscriptor a joint_states ────────────────────────────────────
        self._sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)

        # ── Publisher de markers (latching: RViz recibe aunque llegue tarde) ─
        latch_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._mk_pub = self.create_publisher(
            MarkerArray, '/fkin_markers', latch_qos)

        # ── TF estatico para el frame target_yf ─────────────────────────
        self._static_tf = StaticTransformBroadcaster(self)
        self._broadcast_yf_tf(yf_x, yf_y, yf_z)

        # ── Timer principal ──────────────────────────────────────────────
        dt = 1.0 / max(rate, 0.5)
        self._timer = self.create_timer(dt, self._timer_cb)

        self.get_logger().info(
            f'fkin_display listo.  '
            f'Target yf = ({yf_x:.4f}, {yf_y:.4f}, {yf_z:.4f}) m  '
            f'phi = {yf_phi:.4f} rad')

    # ── Callback de joint_states ─────────────────────────────────────────
    def _js_cb(self, msg: JointState):
        if not self._js_idx:
            self._js_idx = {n: i for i, n in enumerate(msg.name)}
        for k, jname in enumerate(self._JOINT_NAMES):
            idx = self._js_idx.get(jname)
            if idx is not None and idx < len(msg.position):
                self._q[k] = msg.position[idx]

    # ── Timer callback: imprimir + publicar markers ──────────────────────
    def _timer_cb(self):
        x, y, z, phi = open_manx_fkin(*self._q)

        # Impresion en terminal
        self.get_logger().info(
            f'EEF  |  x={x:+.5f} m   y={y:+.5f} m   '
            f'z={z:+.5f} m   phi={phi:+.5f} rad  '
            f'({math.degrees(phi):+.2f} deg)')

        # Publicar markers
        self._publish_markers()

    # ── TF estatico del target yf ────────────────────────────────────────
    def _broadcast_yf_tf(self, x: float, y: float, z: float):
        ts = TransformStamped()
        ts.header.stamp    = self.get_clock().now().to_msg()
        ts.header.frame_id = 'world'
        ts.child_frame_id  = 'target_yf'
        ts.transform.translation.x = x
        ts.transform.translation.y = y
        ts.transform.translation.z = z
        ts.transform.rotation.w    = 1.0   # sin rotacion
        self._static_tf.sendTransform(ts)

    # ── Construccion de markers ──────────────────────────────────────────
    def _arrow_len(self, mid: int, frame: str,
                  dx: float, dy: float, dz: float,
                  r: float, g: float, b: float,
                  length: float) -> Marker:
        """Flecha con longitud explicita."""
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.ns              = 'fkin_axes'
        m.id              = mid
        m.type            = Marker.ARROW
        m.action          = Marker.ADD
        m.scale.x = length * 0.06
        m.scale.y = length * 0.12
        m.scale.z = length * 0.10
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        p0 = Point(); p0.x = p0.y = p0.z = 0.0
        p1 = Point(); p1.x = dx * length; p1.y = dy * length; p1.z = dz * length
        m.points = [p0, p1]
        return m

    def _arrow(self, mid: int, frame: str,
               dx: float, dy: float, dz: float,
               r: float, g: float, b: float) -> Marker:
        """Flecha desde el origen del frame a (dx,dy,dz)*L."""
        return self._arrow_len(mid, frame, dx, dy, dz, r, g, b, self._L)

    def _sphere(self, mid: int, frame: str,
                px: float, py: float, pz: float,
                r: float, g: float, b: float,
                size: float = 0.020) -> Marker:
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.ns              = 'fkin_target'
        m.id              = mid
        m.type            = Marker.SPHERE
        m.action          = Marker.ADD
        m.pose.position.x = px
        m.pose.position.y = py
        m.pose.position.z = pz
        m.pose.orientation.w    = 1.0
        m.scale.x = m.scale.y = m.scale.z = size
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.85
        return m

    def _text(self, mid: int, frame: str,
              px: float, py: float, pz: float,
              text: str) -> Marker:
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.ns              = 'fkin_text'
        m.id              = mid
        m.type            = Marker.TEXT_VIEW_FACING
        m.action          = Marker.ADD
        m.pose.position.x = px
        m.pose.position.y = py
        m.pose.position.z = pz + 0.035
        m.pose.orientation.w = 1.0
        m.scale.z = 0.018
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        m.text    = text
        return m

    def _publish_markers(self):
        ma = MarkerArray()

        # ── Ejes RGB del frame de referencia de la FK: link1 (= world) ───
        # link1 es el frame desde el que se calcula toda la cinematica directa.
        # joint1 esta en (0.012, 0, 0.017) respecto a link1.
        # Escala mayor para que se vea como frame de referencia global.
        L_base = self._L * 1.6
        ma.markers += [
            self._arrow_len(30, 'link1', 1, 0, 0, 1.0, 0.0, 0.0, L_base),
            self._arrow_len(31, 'link1', 0, 1, 0, 0.0, 1.0, 0.0, L_base),
            self._arrow_len(32, 'link1', 0, 0, 1, 0.0, 0.0, 1.0, L_base),
        ]
        ma.markers.append(
            self._text(33, 'link1', 0.0, 0.0, 0.0, 'Base FK\n(link1)'))

        # ── Ejes RGB del end_effector_link (siguen al robot via TF) ──────
        #   X = rojo, Y = verde, Z = azul  (convencion ROS/RViz)
        ma.markers += [
            self._arrow(0, 'end_effector_link', 1, 0, 0,  1.0, 0.0, 0.0),
            self._arrow(1, 'end_effector_link', 0, 1, 0,  0.0, 1.0, 0.0),
            self._arrow(2, 'end_effector_link', 0, 0, 1,  0.0, 0.0, 1.0),
        ]
        ma.markers.append(
            self._text(3, 'end_effector_link', 0.0, 0.0, 0.0, 'EEF'))

        # ── Ejes RGB del frame target_yf (posicion fija) ──────────────────
        ma.markers += [
            self._arrow(10, 'target_yf', 1, 0, 0,  1.0, 0.0, 0.0),
            self._arrow(11, 'target_yf', 0, 1, 0,  0.0, 1.0, 0.0),
            self._arrow(12, 'target_yf', 0, 0, 1,  0.0, 0.0, 1.0),
        ]

        # ── Esfera naranja en yf + etiqueta de texto ──────────────────────
        yf_x, yf_y, yf_z, yf_phi = self._yf
        ma.markers.append(
            self._sphere(20, 'world', yf_x, yf_y, yf_z, 1.0, 0.5, 0.0))
        ma.markers.append(
            self._text(21, 'world', yf_x, yf_y, yf_z,
                       f'yf  ({yf_x:.3f}, {yf_y:.3f}, {yf_z:.3f}) m\n'
                       f'phi = {yf_phi:.3f} rad'))

        self._mk_pub.publish(ma)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = FkinDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
