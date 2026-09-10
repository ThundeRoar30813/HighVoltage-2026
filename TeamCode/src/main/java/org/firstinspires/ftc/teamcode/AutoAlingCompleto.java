package org.firstinspires.ftc.teamcode;

public class AutoAlingCompleto {
    
    // ========== OBJETIVO ==========
    private double targetX = 8;
    private double targetY = 136;
    
    // ========== POSICIÓN DEL ROBOT ==========
    private double robotX = 9;
    private double robotY = 9;
    private double robotHeading = 90;  // radianes
    
    // ========== PARÁMETROS ==========
    private final double ZONA_MUERTA = Math.toRadians(3);   // 3 grados
    private final double VEL_MIN = 0.1;
    private final double VEL_MAX = 1.0;
    private final double VEL_MAX_ERROR = Math.toRadians(90);
    
    // ========== ACTUALIZAR POSICIÓN ==========
    public void updateRobot(double x, double y, double headingRadianes) {
        this.robotX = x;
        this.robotY = y;
        this.robotHeading = headingRadianes;
    }
    
    // ========== CAMBIAR OBJETIVO ==========
    public void setTargetAzul() {
        targetX = 8;
        targetY = 136;
    }
    
    public void setTargetRojo() {
        targetX = 136;
        targetY = 8;
    }
    
    // ========== CALCULAR EL GIRO (devuelve entre -1 y 1) ==========
    public double calcularGiro() {
        
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        
        if (Math.abs(deltaX) < 0.1 && Math.abs(deltaY) < 0.1) {
            return 0;
        }
        
        double anguloObjetivo = Math.atan2(deltaY, deltaX);
        double error = anguloObjetivo - robotHeading;
        error = Math.atan2(Math.sin(error), Math.cos(error));
        
        if (Math.abs(error) < ZONA_MUERTA) {
            return 0;
        }
        
        double velocidad = Math.abs(error) / VEL_MAX_ERROR;
        if (velocidad < VEL_MIN) velocidad = VEL_MIN;
        if (velocidad > VEL_MAX) velocidad = VEL_MAX;
        
        return velocidad * Math.signum(error);
    }
    
    // ========== APLICAR EL GIRO AL ROBOT ==========
    // Este método SIMULA el giro del robot
    // En un robot real, aquí enviarías la potencia a los motores
    public void aplicarGiro(double giro) {
        // SIMULAR giro: la orientación del robot cambia según el giro
        // En un robot REAL, esto NO va aquí, va en los motores
        robotHeading += giro * 0.05;  // 0.05 es la sensibilidad de giro
        robotHeading = Math.atan2(Math.sin(robotHeading), Math.cos(robotHeading));
    }
    
    // ========== MOVIMIENTO MANUAL (para prueba) ==========
    public void moverManual(double adelante, double strafe, double giroManual) {
        // Simular movimiento con joysticks
        robotX += adelante * 0.5;
        robotY += strafe * 0.5;
        robotHeading += giroManual * 0.05;
        robotHeading = Math.atan2(Math.sin(robotHeading), Math.cos(robotHeading));
    }
    
    // ========== GETTERS PARA TELEMETRÍA ==========
    public double getRobotX() { return robotX; }
    public double getRobotY() { return robotY; }
    public double getRobotHeadingDeg() { return Math.toDegrees(robotHeading); }
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getDistancia() {
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        return Math.sqrt(dx*dx + dy*dy);
    }
    public double getErrorGrados() {
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        double anguloObjetivo = Math.atan2(deltaY, deltaX);
        double error = anguloObjetivo - robotHeading;
        error = Math.atan2(Math.sin(error), Math.cos(error));
        return Math.toDegrees(error);
    }
    public boolean isAlineado() {
        return Math.abs(getErrorGrados()) < 3;
    }
}