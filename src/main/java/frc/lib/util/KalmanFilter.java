package frc.lib.util;

public class KalmanFilter {
    private double estimate, errorCovariance;
    private final double processNoise, sensorNoise;

    public KalmanFilter(double processNoise, double sensorNoise) {
        this.processNoise = processNoise;
        this.sensorNoise = sensorNoise;
        this.estimate = 0.0;
        this.errorCovariance = 1.0;
    }
    
    public double update(double measurement) {
        errorCovariance += processNoise;

        double kalmanGain = errorCovariance / (errorCovariance + sensorNoise);
        estimate += kalmanGain * (measurement - estimate);
        errorCovariance *= (1 - kalmanGain);

        return estimate;
    }

    public double getEstimate() {
        return estimate;
    }
}
