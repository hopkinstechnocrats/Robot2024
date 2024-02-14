package frc.robot.commands;

/*needed things:
getSelectedSensorposition() * DriveConstants.kEncoderDistancePerPulse;
*/

/*public class DistanceDrive {

   public DistanceDrive() {}
   public void getSelectedSensorposition(){
    //SwerveControlRequestParameters.setSelectedSensorPosition;
   }
   public static Command distanceDrive(Drive drive) {
   return Commands.run(
   () -> {
  double speed = 0.5;
  double driveDistance = speed * 5; // five seconds of driving
   //double distanceTraveled = getSelectedSensorposition() * DriveConstants.kEncoderDistancePerPulse;

   //while (distanceTraveled < driveDistance) {
   drive.runVelocity(
   ChassisSpeeds.fromFieldRelativeSpeeds(speed,
   0,
   0,
   drive.getRotation()));
//}

         //});
   //}
 //}*/
