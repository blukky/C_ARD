








int limited(int a, float min, float max){
    if (a >= max){
        return max;
    }else if (a <= min){
        return min;
    }
    return a;
}

float *pid_function(float k_p, float k_i, float k_d, float dt, float integral, float error, float last_error, float min, float max) {
    float out[3];
    integral = integral + error * dt;
    integral = limited(integral, min, max);
    float output = k_p * error + integral * k_i + k_d * ((error - last_error) / dt);
    output = limited(output, min, max);
    out[0] = output;
    out[1] = error;
    out[2] = integral;
    return out;
}