// Entradas
float d_left;   // Distancia rueda izquierda
float d_right;  // Distancia rueda derecha
float L;        // Distancia entre ruedas

// Estado del robot
float x = 0, y = 0, theta = 0; // Posición y orientación inicial

// Paso de odometría
float d_center = (d_left + d_right) / 2.0;
float d_theta = (d_right - d_left) / L;

theta += d_theta;
x += d_center * cos(theta);
y += d_center * sin(theta);
