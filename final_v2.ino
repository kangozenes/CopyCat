int* receivedChars;// char receivedChars[numChars];
int* receivedVec;
int** mat;  
int w,h; // width, height of matrix
int dim[2];

int motor1pin1 = 2;
int motor1pin2 = 3;
int motor2pin1 = 4;
int motor2pin2 = 5;
int p=0;
int ndi = 0;

boolean newData = false;
int startMarker = 253; //char startMarker = '<';
int endMarker = 254; //char endMarker = '>';

void setup() {
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
    pinMode(motor1pin1, OUTPUT);
    pinMode(motor1pin2, OUTPUT);
    pinMode(motor2pin1, OUTPUT);
    pinMode(motor2pin2, OUTPUT);

    pinMode(9, OUTPUT); 
    pinMode(10, OUTPUT);
    w=3; // X,Y,Velocity
}

void loop() {
   Init();
  //  receivedVector = recvWithStartEndMarkers();
   // vec2mat(receivedVector);
    recv();
    driver();
    showNewData(); 
}

int Init(){
  int matinfo; 

  if(ndi!=2){
    Serial.println("Initialization..."); 
    while (Serial.available() > 0 && newData == false){
      static boolean recvInProgress = false;
      
      matinfo = Serial.read();
  
          if (recvInProgress == true) {
              if (matinfo != endMarker) { 
                  dim[ndi] = matinfo;
                  ndi++;
              }
              else {
                  recvInProgress = false;
                  ndi = 0;
                  newData = true;
              }
          }
          else if (matinfo == startMarker) {
              recvInProgress = true;
          }
          
          if (ndi==2){     
            h = dim[0];
            w = dim[1];
            
            Serial.print(sprintf("Height = %d and Width = %d",h,w)); 
            break;
            }
    }
  }
}


int recv() {
    static boolean recvInProgress = false;
    static int ndx = 0;
   
    int rc; //char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) { 
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                ndi = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
   // return receivedChars;
}

void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");     
        for(int i = 0;i<255;i++){
        Serial.println(receivedChars[i]); 
        }
        /*
        for(int i = 0;i<3;i++){
          for(int j = 0;j<3;j++){
        Serial.println(mat[i][j]);
          }
        }
        */
        newData = false;
    }
}


void vec2mat(int* vec){
  int k =0;
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
       mat[i][j] = *(vec+k);
       k++;
    }
  }
 }



void driver(){
  if (p == 105){ p=0; }
  
    if(receivedChars[p]==3&&p<105)
    {
      analogWrite(9,123 ); //ENA pin SOL MOTOR
      analogWrite(10,128 ); //ENB pin SAĞ MOTOR
      delay(250);
      }
    else if(receivedChars[p]==1&&p<105)
    {
      analogWrite(9,180 ); //ENA pin SOL MOTOR
      analogWrite(10, 128 ); //ENB pin SAĞ MOTOR
      delay(250);
      }
    
    else if(receivedChars[p]==2&&p<105)
    {
      analogWrite(9,123); //ENA pin SOL MOTOR
      analogWrite(10, 190 ); //ENB pin SAĞ MOTOR
      delay(250);
      }
    
     else if(receivedChars[p]==0&&p<105){
      analogWrite(9,0); //ENA pin SOL MOTOR
      analogWrite(10, 0); //ENB pin SAĞ MOTOR
        }
       p++;
      //Controlling speed (0 = off and 255 = max speed):

      //Controlling spin direction of motors:
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, HIGH);
    
      digitalWrite(motor2pin1, HIGH);
      digitalWrite(motor2pin2, LOW);
 
}
