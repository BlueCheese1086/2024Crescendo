package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

// This class allows you to use constants from your Constants.java file and 
public class YAGSLConverter {
    public static class Configs {
        public static class SwerveDriveJson {
            public static class IMU {
                public static String type = "pigeon";
                public static int id = 0;
                public static String canbus = "";
            }

            public static boolean invertedIMU = false;
        }

        public static class PhysicalPropertiesJson {
            public static double optimalVoltage = 12;
            public static double wheelGripCoefficientOfFriction = 1.19;
            
            public static class CurrentLimit {
                public static double drive = 40;
                public static double angle = 20;
            }

            public static class ConversionFactors {
                public class Angle {
                    public static double gearRatio = 0;
                    public static double factor = 1;
                }

                public class Drive {
                    public static double diameter = 2;
                    public static double gearRatio = 0;
                    public static double factor = 1;
                }
            }

            public static class RampRate {
                public static double drive = 0.25;
                public static double angle = 0.25;
            }
        }

        public static class FrontLeftModuleJson {
            public static class Location {
                public static double front = 0;
                public static double left = 0;
            }

            public static double absoluteEncoderOffset = 0;

            public static class Drive {
                public static String type = "sparkflex";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Angle {
                public static String type = "sparkflex";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Encoder {
                public static String type = "none";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Inverted {
                public static boolean drive = false; 
                public static boolean angle = false;
            }

            public static boolean absoluteEncoderInverted = false;
        }

        public static class FrontRightModuleJson {
            public static class Location {
                public static double front = 0;
                public static double left = 0;
            }

            public static double absoluteEncoderOffset = 0;

            public static class Drive {
                public static String type = "sparkflex";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Angle {
                public static String type = "sparkflex";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Encoder {
                public static String type = "none";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Inverted {
                public static boolean drive = false; 
                public static boolean angle = false;
            }

            public static boolean absoluteEncoderInverted = false;
        }

        public static class BackLeftModuleJson {
            public static class Location {
                public static double front = 0;
                public static double left = 0;
            }

            public static double absoluteEncoderOffset = 0;

            public static class Drive {
                public static String type = "sparkflex";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Angle {
                public static String type = "sparkflex";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Encoder {
                public static String type = "none";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Inverted {
                public static boolean drive = false; 
                public static boolean angle = false;
            }

            public static boolean absoluteEncoderInverted = false;
        }

        public static class BackRightModuleJson {
            public static class Location {
                public static double front = 0;
                public static double left = 0;
            }

            public static double absoluteEncoderOffset = 0;

            public static class Drive {
                public static String type = "sparkflex";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Angle {
                public static String type = "sparkflex";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Encoder {
                public static String type = "none";
                public static int id = 0;
                public static String canbus = "";
            }

            public static class Inverted {
                public static boolean drive = false; 
                public static boolean angle = false;
            }

            public static boolean absoluteEncoderInverted = false;
        }

        public static class ControllerPropertiesJson {
            public static double angleJoystickRadiusDeadband = 0.5;

            public static class Heading {
                public static double kP = 0.4;
                public static double kI = 0;
                public static double kD = 0.01;
            }
        }

        public static class PIDFPropertiesJson {
            public static class Drive {
                public static double kP = 0;
                public static double kI = 0;
                public static double kD = 0;
                public static double staticFeedForward = 0;
                public static double integralZone = 0;
            }

            public static class Angle {
                public static double kP = 0;
                public static double kI = 0;
                public static double kD = 0;
                public static double staticFeedForward = 0;
                public static double integralZone = 0;
            }
        }

    }

    private String deployPath;

    public YAGSLConverter() {
        deployPath = "./src/main/deploy";
    }

    public void writeConfigs() {
        try {
            FileWriter writer = new FileWriter(deployPath + "/swerve/swervedrive.json");

            writer.write(String.format("{\n"));
            writer.write(String.format("  \"imu\": {\n"));
            writer.write(String.format("    \"type\": \"%s\",\n", Configs.SwerveDriveJson.IMU.type));
            writer.write(String.format("    \"id\": %s,\n", Configs.SwerveDriveJson.IMU.id));
            writer.write(String.format("    \"canbus\": \"%s\"\n", Configs.SwerveDriveJson.IMU.canbus));
            writer.write(String.format("  },\n"));
            writer.write(String.format("  \"invertedIMU\": %s,\n", Configs.SwerveDriveJson.invertedIMU));
            writer.write(String.format("  \"modules\": [\n"));
            writer.write(String.format("    \"frontleft.json\",\n"));
            writer.write(String.format("    \"frontright.json\",\n"));
            writer.write(String.format("    \"backleft.json\",\n"));
            writer.write(String.format("    \"backright.json\"\n"));
            writer.write(String.format("  ]\n"));
            writer.write(String.format("}\n"));

            writer.close();
        } catch (IOException err) {
            System.out.println("Error writing to swervedrive.json");
        }

        try {
            FileWriter writer = new FileWriter(deployPath + "/swerve/modules/physicalproperties.json");

            writer.write(String.format("{\n"));
            writer.write(String.format("    \"optimalVoltage\": %s,\n", Configs.PhysicalPropertiesJson.optimalVoltage));
            writer.write(String.format("    \"wheelGripCoefficientOfFriction\": %s,\n", Configs.PhysicalPropertiesJson.wheelGripCoefficientOfFriction));
            writer.write(String.format("    \"currentLimit\": {\n"));
            writer.write(String.format("        \"drive\": %s,\n", Configs.PhysicalPropertiesJson.CurrentLimit.drive));
            writer.write(String.format("        \"angle\": %s\n", Configs.PhysicalPropertiesJson.CurrentLimit.angle));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"conversionFactors\": {\n"));
            writer.write(String.format("        \"angle\": {\n"));
            writer.write(String.format("            \"gearRatio\": %s,\n", Configs.PhysicalPropertiesJson.ConversionFactors.Angle.gearRatio));
            writer.write(String.format("            \"factor\": %s\n", Configs.PhysicalPropertiesJson.ConversionFactors.Angle.factor));
            writer.write(String.format("        },\n"));
            writer.write(String.format("        \"drive\": {\n"));
            writer.write(String.format("            \"diameter\": %s,\n", Configs.PhysicalPropertiesJson.ConversionFactors.Drive.diameter));
            writer.write(String.format("            \"gearRatio\": %s,\n", Configs.PhysicalPropertiesJson.ConversionFactors.Drive.gearRatio));
            writer.write(String.format("            \"factor\": %s\n", Configs.PhysicalPropertiesJson.ConversionFactors.Drive.factor));
            writer.write(String.format("        }\n"));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"rampRate\": {\n"));
            writer.write(String.format("        \"drive\": %s,\n", Configs.PhysicalPropertiesJson.RampRate.drive));
            writer.write(String.format("        \"angle\": %s\n", Configs.PhysicalPropertiesJson.RampRate.angle));
            writer.write(String.format("    }"));
            writer.write(String.format("}"));

            writer.close();
        } catch (IOException err) {
            System.out.println("Error writing to physicalproperties.json");
        }

        try {
            FileWriter writer = new FileWriter(deployPath + "/swerve/modules/frontleft.json");

            writer.write(String.format("{\n"));
            writer.write(String.format("    \"location\": {\n"));
            writer.write(String.format("        \"front\": %s,\n", Configs.FrontLeftModuleJson.Location.front));
            writer.write(String.format("        \"left\": %s\n", Configs.FrontLeftModuleJson.Location.left));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"absoluteEncoderOffset\": %s,\n", Configs.FrontLeftModuleJson.absoluteEncoderOffset));
            writer.write(String.format("    \"drive\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.FrontLeftModuleJson.Drive.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.FrontLeftModuleJson.Drive.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.FrontLeftModuleJson.Drive.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"angle\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.FrontLeftModuleJson.Angle.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.FrontLeftModuleJson.Angle.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.FrontLeftModuleJson.Angle.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"encoder\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.FrontLeftModuleJson.Encoder.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.FrontLeftModuleJson.Encoder.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.FrontLeftModuleJson.Encoder.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"inverted\": {\n"));
            writer.write(String.format("        \"drive\": %s,\n", Configs.FrontLeftModuleJson.Inverted.drive));
            writer.write(String.format("        \"angle\": %s\n", Configs.FrontLeftModuleJson.Inverted.angle));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"absoluteEncoderInverted\": %s\n", Configs.FrontLeftModuleJson.absoluteEncoderInverted));
            writer.write(String.format("}\n"));

            writer.close();
        } catch (IOException err) {
            System.out.println("Error writing to frontleft.json");
        }

    try {
            FileWriter writer = new FileWriter(deployPath + "/swerve/modules/frontright.json");

            writer.write(String.format("{\n"));
            writer.write(String.format("    \"location\": {\n"));
            writer.write(String.format("        \"front\": %s,\n", Configs.FrontRightModuleJson.Location.front));
            writer.write(String.format("        \"left\": %s\n", Configs.FrontRightModuleJson.Location.left));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"absoluteEncoderOffset\": %s,\n", Configs.FrontRightModuleJson.absoluteEncoderOffset));
            writer.write(String.format("    \"drive\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.FrontRightModuleJson.Drive.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.FrontRightModuleJson.Drive.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.FrontRightModuleJson.Drive.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"angle\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.FrontRightModuleJson.Angle.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.FrontRightModuleJson.Angle.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.FrontRightModuleJson.Angle.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"encoder\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.FrontRightModuleJson.Encoder.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.FrontRightModuleJson.Encoder.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.FrontRightModuleJson.Encoder.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"inverted\": {\n"));
            writer.write(String.format("        \"drive\": %s,\n", Configs.FrontRightModuleJson.Inverted.drive));
            writer.write(String.format("        \"angle\": %s\n", Configs.FrontRightModuleJson.Inverted.angle));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"absoluteEncoderInverted\": %s\n", Configs.FrontRightModuleJson.absoluteEncoderInverted));
            writer.write(String.format("}\n"));

            writer.close();
        } catch (IOException err) {
            System.out.println("Error writing to frontright.json");
        }

        try {
            FileWriter writer = new FileWriter(deployPath + "/swerve/modules/backleft.json");

            writer.write(String.format("{\n"));
            writer.write(String.format("    \"location\": {\n"));
            writer.write(String.format("        \"front\": %s,\n", Configs.BackLeftModuleJson.Location.front));
            writer.write(String.format("        \"left\": %s\n", Configs.BackLeftModuleJson.Location.left));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"absoluteEncoderOffset\": %s,\n", Configs.BackLeftModuleJson.absoluteEncoderOffset));
            writer.write(String.format("    \"drive\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.BackLeftModuleJson.Drive.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.BackLeftModuleJson.Drive.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.BackLeftModuleJson.Drive.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"angle\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.BackLeftModuleJson.Angle.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.BackLeftModuleJson.Angle.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.BackLeftModuleJson.Angle.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"encoder\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.BackLeftModuleJson.Encoder.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.BackLeftModuleJson.Encoder.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.BackLeftModuleJson.Encoder.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"inverted\": {\n"));
            writer.write(String.format("        \"drive\": %s,\n", Configs.BackLeftModuleJson.Inverted.drive));
            writer.write(String.format("        \"angle\": %s\n", Configs.BackLeftModuleJson.Inverted.angle));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"absoluteEncoderInverted\": %s\n", Configs.BackLeftModuleJson.absoluteEncoderInverted));
            writer.write(String.format("}\n"));

            writer.close();
        } catch (IOException err) {
            System.out.println("Error writing to backleft.json");
        }

        try {
            FileWriter writer = new FileWriter(deployPath + "/swerve/modules/backright.json");

            writer.write(String.format("{\n"));
            writer.write(String.format("    \"location\": {\n"));
            writer.write(String.format("        \"front\": %s,\n", Configs.BackRightModuleJson.Location.front));
            writer.write(String.format("        \"left\": %s\n", Configs.BackRightModuleJson.Location.left));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"absoluteEncoderOffset\": %s,\n", Configs.BackRightModuleJson.absoluteEncoderOffset));
            writer.write(String.format("    \"drive\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.BackRightModuleJson.Drive.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.BackRightModuleJson.Drive.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.BackRightModuleJson.Drive.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"angle\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.BackRightModuleJson.Angle.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.BackRightModuleJson.Angle.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.BackRightModuleJson.Angle.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"encoder\": {\n"));
            writer.write(String.format("        \"type\": \"%s\",\n", Configs.BackRightModuleJson.Encoder.type));
            writer.write(String.format("        \"id\": %s,\n", Configs.BackRightModuleJson.Encoder.id));
            writer.write(String.format("        \"canbus\": \"%s\"\n", Configs.BackRightModuleJson.Encoder.canbus));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"inverted\": {\n"));
            writer.write(String.format("        \"drive\": %s,\n", Configs.BackRightModuleJson.Inverted.drive));
            writer.write(String.format("        \"angle\": %s\n", Configs.BackRightModuleJson.Inverted.angle));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"absoluteEncoderInverted\": %s\n", Configs.BackRightModuleJson.absoluteEncoderInverted));
            writer.write(String.format("}\n"));

            writer.close();
        } catch (IOException err) {
            System.out.println("Error writing to backright.json");
        }

        try {
            FileWriter writer = new FileWriter(deployPath + "/swerve/controllerproperties.json");

            writer.write(String.format("{\n"));
            writer.write(String.format("    \"angleJoystickRadiusDeadband\": %s,\n", Configs.ControllerPropertiesJson.angleJoystickRadiusDeadband));
            writer.write(String.format("    \"heading\": {\n"));
            writer.write(String.format("        \"p\": %s,\n", Configs.ControllerPropertiesJson.Heading.kP));
            writer.write(String.format("        \"i\": %s,\n", Configs.ControllerPropertiesJson.Heading.kI));
            writer.write(String.format("        \"d\": %s\n", Configs.ControllerPropertiesJson.Heading.kD));
            writer.write(String.format("    }\n"));
            writer.write(String.format("}\n"));

            writer.close();
        } catch (IOException err) {
            System.out.println("Error writing to controllerproperties.json");
        }

        try {
            FileWriter writer = new FileWriter(deployPath + "/swerve/modules/pidfproperties.json");

            writer.write(String.format("{\n"));
            writer.write(String.format("    \"drive\": {\n"));
            writer.write(String.format("        \"p\": %s,\n", Configs.PIDFPropertiesJson.Drive.kP));
            writer.write(String.format("        \"i\": %s,\n", Configs.PIDFPropertiesJson.Drive.kI));
            writer.write(String.format("        \"d\": %s,\n", Configs.PIDFPropertiesJson.Drive.kD));
            writer.write(String.format("        \"f\": %s,\n", Configs.PIDFPropertiesJson.Drive.staticFeedForward));
            writer.write(String.format("        \"iz\": %s\n", Configs.PIDFPropertiesJson.Drive.integralZone));
            writer.write(String.format("    },\n"));
            writer.write(String.format("    \"angle\": {\n"));
            writer.write(String.format("        \"p\": %s,\n", Configs.PIDFPropertiesJson.Angle.kP));
            writer.write(String.format("        \"i\": %s,\n", Configs.PIDFPropertiesJson.Angle.kI));
            writer.write(String.format("        \"d\": %s,\n", Configs.PIDFPropertiesJson.Angle.kD));
            writer.write(String.format("        \"f\": %s,\n", Configs.PIDFPropertiesJson.Angle.staticFeedForward));
            writer.write(String.format("        \"iz\": %s\n", Configs.PIDFPropertiesJson.Angle.integralZone));
            writer.write(String.format("    }\n"));
            writer.write(String.format("}\n"));                    

            writer.close();
        } catch (IOException err) {
            System.out.println("Error writing to pidfproperties.json");
        }
    }
}