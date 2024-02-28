package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;

public class AutoFinder {
    public static String getFirstPathName(String autoName) {
        JSONParser parser = new JSONParser();

        try {
            Reader reader = new FileReader(new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/pathplanner/autos/" + autoName + ".auto"));

            for (Object obj : (JSONArray) ((JSONObject) ((JSONObject) ((JSONObject) parser.parse(reader)).get("command")).get("data")).get("commands")) {
                String val = (String) ((JSONObject) obj).get("type");
                if (val.equals("path")) {
                    reader.close();
                    return (String) ((JSONObject) ((JSONObject) obj).get("data")).get("pathName");
                }
            };
        } catch (FileNotFoundException err) {
            System.out.println("Could not file " + autoName + ".");
        } catch (ParseException err) {
            System.out.println("There wan an error parsing the JSON.");
        } catch (IOException err) {
            System.out.println("There wan an error parsing the JSON.");
        }

        return "";
    }

    public static Pose2d getInitPoseFromPathName(String pathName) {
        try {
            Reader reader = new FileReader(new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/pathplanner/paths/" + pathName + ".path"));
            JSONParser parser = new JSONParser();
            JSONObject obj = (JSONObject) parser.parse(reader);
            double x = (Double) ((JSONObject) ((JSONObject) ((JSONArray) obj.get("waypoints")).get(0)).get("anchor")).get("x");
            double y = (Double) ((JSONObject) ((JSONObject) ((JSONArray) obj.get("waypoints")).get(0)).get("anchor")).get("y");

            return new Pose2d(x, y, new Rotation2d());
        } catch (FileNotFoundException err) {
            System.out.println("Could not file " + pathName + ".");
        } catch (ParseException err) {
            System.out.println("There wan an error parsing the JSON.");
        } catch (IOException err) {
            System.out.println("There wan an error parsing the JSON.");
        }

        return new Pose2d();
    }
}
