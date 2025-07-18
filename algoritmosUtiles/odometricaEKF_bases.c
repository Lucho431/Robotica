/////////////////////////////////// ESTADO DEL ROBOT ///////////////////////////////////////////////
// Estado estimado: x = [x, y, theta]
float x_est[3];  // [x, y, θ]

// Covarianza del estado (incertidumbre)
float P[3][3];   // Matriz 3x3


////////////////////////////////// MODELO DE MOVIMIENTO DIFERENCIAL //////////////////////////////////////
// Entradas (control)
float d_left, d_right;
float L; // distancia entre ruedas

float d_center = (d_left + d_right) / 2.0;
float d_theta = (d_right - d_left) / L;

float theta = x_est[2];

float dx = d_center * cos(theta);
float dy = d_center * sin(theta);


/////////////////////////////////////////// PASO 1: PREDICCION ////////////////////////////////////////////////////////
// Predicción del estado
x_est[0] += dx;
x_est[1] += dy;
x_est[2] += d_theta;

// Predicción de la covarianza
// P = F*P*F^T + Q
// Donde:
// F es el Jacobiano del modelo de movimiento respecto al estado
// Q es la matriz de ruido del modelo (estimada)


///////////////////////////////////////// PASO 2: CORRECCION (SI SE TIENEN SENSORES EXTERNOS) ///////////////////////////////////////
//Si tenés un IMU (por ejemplo, giroscopio que mide θ), podés usarlo para corregir:
float z_theta = medida_gyro;  // medición de orientación
float y_theta = z_theta - x_est[2];  // innovación

// Kalman gain
float S = P[2][2] + R_theta; // R_theta es la varianza del ruido del giroscopio
float K = P[2][2] / S;

// Corrección
x_est[2] += K * y_theta;
P[2][2] = (1 - K) * P[2][2];
