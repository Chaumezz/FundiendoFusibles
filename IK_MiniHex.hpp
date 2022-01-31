//***********************************************************************
// MiniHex IK Routine
//***********************************************************************

//output to the appropriate leg
switch (leg_number)
{
case 0:
  if (leg1_IK_control == true)                      //flag for IK or manual control of leg
  {
    theta_coxa = theta_coxa + 90.0;                 //compensate for leg mounting
    theta_coxa = constrain(theta_coxa, 0.0, 180.0);
    theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
    coxa1_servo.write(int(theta_coxa));
    theta_femur = map(theta_femur, 0, 180, 180, 0); //inverted servo
    femur1_servo.write(int(theta_femur));
    tibia1_servo.write(int(theta_tibia));
  }
  break;
case 1:
  theta_coxa = theta_coxa + 90.0;                 //compensate for leg mounting
  theta_coxa = constrain(theta_coxa, 0.0, 180.0);
  theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
  coxa2_servo.write(int(theta_coxa));
  theta_femur = map(theta_femur, 0, 180, 180, 0); //inverted servo
  femur2_servo.write(int(theta_femur));
  tibia2_servo.write(int(theta_tibia));
  break;
case 2:
  theta_coxa = theta_coxa + 90.0;                 //compensate for leg mounting
  theta_coxa = constrain(theta_coxa, 0.0, 180.0);
  theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
  coxa3_servo.write(int(theta_coxa));
  theta_femur = map(theta_femur, 0, 180, 180, 0); //inverted servo
  femur3_servo.write(int(theta_femur));
  tibia3_servo.write(int(theta_tibia));
  break;
case 3:
  if (theta_coxa < 0)                               //compensate for leg mounting
    theta_coxa = theta_coxa + 270.0;                // (need to use different
  else                                              //  positive and negative offsets
    theta_coxa = theta_coxa - 90.0;                //  due to atan2 results above!)
  theta_coxa = constrain(theta_coxa, 0.0, 180.0);
  theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
  coxa4_servo.write(int(theta_coxa));
  femur4_servo.write(int(theta_femur));
  theta_tibia = map(theta_tibia, 0, 180, 180, 0); //inverted servo
  tibia4_servo.write(int(theta_tibia));
  break;
case 4:
  if (theta_coxa < 0)                               //compensate for leg mounting
    theta_coxa = theta_coxa + 270.0;                // (need to use different
  else                                              //  positive and negative offsets
    theta_coxa = theta_coxa - 90.0;                 //  due to atan2 results above!)
  theta_coxa = constrain(theta_coxa, 0.0, 180.0);
  theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
  coxa5_servo.write(int(theta_coxa));
  femur5_servo.write(int(theta_femur));
  theta_tibia = map(theta_tibia, 0, 180, 180, 0); //inverted servo
  tibia5_servo.write(int(theta_tibia));
  break;
case 5:
  if (leg6_IK_control == true)                      //flag for IK or manual control of leg
  {
    if (theta_coxa < 0)                             //compensate for leg mounting
      theta_coxa = theta_coxa + 270.0;              // (need to use different
    else                                            //  positive and negative offsets
      theta_coxa = theta_coxa - 90.0;               //  due to atan2 results above!)
    theta_coxa = constrain(theta_coxa, 0.0, 180.0);
    theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
    coxa6_servo.write(int(theta_coxa));
    femur6_servo.write(int(theta_femur));
    theta_tibia = map(theta_tibia, 0, 180, 180, 0); //inverted servo
    tibia6_servo.write(int(theta_tibia));
  }
  break;
}
