// Estructuras
typedef struct {
    float x, y, theta;
} RobotState;

float P[3][3];   // Matriz de covarianza del estado
float Q[3][3];   // Ruido del proceso
float R;         // Varianza del sensor (IMU)

// Paso de predicción
void kalman_predict(RobotState *state, float d_left, float d_right, float b) {
    float ds = 0.5f * (d_right + d_left);
    float dtheta = (d_right - d_left) / b;

    float theta_mid = state->theta + 0.5f * dtheta;

    state->x     += ds * cosf(theta_mid);
    state->y     += ds * sinf(theta_mid);
    state->theta += dtheta;

    // Normalizar ángulo entre -pi y pi
    while (state->theta > M_PI) state->theta -= 2 * M_PI;
    while (state->theta < -M_PI) state->theta += 2 * M_PI;

    // Aquí iría la actualización de la matriz P con el Jacobiano F y Q
}

// Paso de corrección con IMU (medición de orientación)
void kalman_update(RobotState *state, float theta_measured) {
    float y = theta_measured - state->theta; // Innovación
    while (y > M_PI) y -= 2 * M_PI;
    while (y < -M_PI) y += 2 * M_PI;

    float S = P[2][2] + R;         // Innovación de covarianza
    float K = P[2][2] / S;         // Ganancia de Kalman (solo para theta)

    state->theta += K * y;

    // Normalizar
    while (state->theta > M_PI) state->theta -= 2 * M_PI;
    while (state->theta < -M_PI) state->theta += 2 * M_PI;

    P[2][2] *= (1 - K);            // Actualización de covarianza
}
