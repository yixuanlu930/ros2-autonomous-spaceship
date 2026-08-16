# Estado completo de la nave (publicado a 20 Hz)
std_msgs/Header header

float64 x            # posición X (metros)
float64 y            # posición Y (metros)
float64 heading      # orientación (radianes): 0=este, pi/2=norte
float64 vx           # velocidad en X (m/s)
float64 vy           # velocidad en Y (m/s)
float64 omega        # velocidad angular (rad/s)

float32 power_m1     # potencia actual motor izquierdo (0-100)
float32 power_m2     # potencia actual motor derecho (0-100)

float64 elapsed_time # tiempo desde primer impulso hasta llegada (s)
bool    arrived      # True cuando dist<2m y speed<0.1 m/s
float64 target_x     # target actual X (metros)
float64 target_y     # target actual Y (metros)
