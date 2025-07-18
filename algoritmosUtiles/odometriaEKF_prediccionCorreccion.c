///////////////////////// ESTADO Y MATRICES ///////////////////////////////
#define STATE_SIZE 3
typedef struct {
    float x[STATE_SIZE];     // [x, y, theta]
    float P[STATE_SIZE][STATE_SIZE];
} ekf_t;


/////////////////////// PREDICCION /////////////////////////////////////
void ekf_predict(ekf_t *ekf, float v_l, float v_r, float L, float dt, float Q[STATE_SIZE][STATE_SIZE]) {
    float v = (v_r + v_l) / 2.0f;
    float w = (v_r - v_l) / L;
    float theta = ekf->x[2];

    // Predicción de estado
    ekf->x[0] += v * cosf(theta) * dt;
    ekf->x[1] += v * sinf(theta) * dt;
    ekf->x[2] += w * dt;

    // Normalización del ángulo
    while (ekf->x[2] > M_PI) ekf->x[2] -= 2 * M_PI;
    while (ekf->x[2] < -M_PI) ekf->x[2] += 2 * M_PI;

    // F (Jacobiano de la predicción)
    float F[STATE_SIZE][STATE_SIZE] = {
        {1, 0, -v * sinf(theta) * dt},
        {0, 1,  v * cosf(theta) * dt},
        {0, 0, 1}
    };

    // P = F * P * Fᵗ + Q
    float P_tmp[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; i++)
        for (int j = 0; j < STATE_SIZE; j++)
            for (int k = 0; k < STATE_SIZE; k++)
                P_tmp[i][j] += F[i][k] * ekf->P[k][j];

    float FT[STATE_SIZE][STATE_SIZE] = {
        {F[0][0], F[1][0], F[2][0]},
        {F[0][1], F[1][1], F[2][1]},
        {F[0][2], F[1][2], F[2][2]},
    };

    float P_new[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; i++)
        for (int j = 0; j < STATE_SIZE; j++)
            for (int k = 0; k < STATE_SIZE; k++)
                P_new[i][j] += P_tmp[i][k] * FT[k][j];

    for (int i = 0; i < STATE_SIZE; i++)
        for (int j = 0; j < STATE_SIZE; j++)
            ekf->P[i][j] = P_new[i][j] + Q[i][j];
}


////////////////////////////// CORRECCION ///////////////////////////////////////
void ekf_correct_angle(ekf_t *ekf, float theta_measured, float R_theta) {
    float y = theta_measured - ekf->x[2];

    // Normalizar ángulo
    while (y > M_PI) y -= 2 * M_PI;
    while (y < -M_PI) y += 2 * M_PI;

    float H[STATE_SIZE] = {0, 0, 1};  // Sólo medimos theta
    float H_P[STATE_SIZE] = {0};
    float S = 0;
    for (int i = 0; i < STATE_SIZE; i++) {
        H_P[i] = H[0]*ekf->P[0][i] + H[1]*ekf->P[1][i] + H[2]*ekf->P[2][i];
        S += H_P[i] * H[i];
    }
    S += R_theta;

    float K[STATE_SIZE];
    for (int i = 0; i < STATE_SIZE; i++)
        K[i] = H_P[i] / S;

    // Actualización del estado
    for (int i = 0; i < STATE_SIZE; i++)
        ekf->x[i] += K[i] * y;

    // Actualización de la covarianza
    float KH[STATE_SIZE][STATE_SIZE] = {0};
    for (int i = 0; i < STATE_SIZE; i++)
        for (int j = 0; j < STATE_SIZE; j++)
            KH[i][j] = K[i] * H[j];

    for (int i = 0; i < STATE_SIZE; i++)
        for (int j = 0; j < STATE_SIZE; j++)
            ekf->P[i][j] -= KH[i][j] * ekf->P[j][j]; // Simplificado
}

