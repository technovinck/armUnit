#ifndef WEBPAGE_H
#define WEBPAGE_H

const char* rootPage = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>Servo Control</title>
</head>
<body>
  <h1>Servo Control</h1>
  <form action='/setServo' method='POST'>
    Servo 0: <input type='text' name='servo0'><br>
    Servo 1: <input type='text' name='servo1'><br>
    Servo 2: <input type='text' name='servo2'><br>
    Servo 3: <input type='text' name='servo3'><br>
    <input type='submit' value='Set Servo Positions'>
  </form>
</body>
</html>
)=====";

#endif
