/*  Appendix C: Robocone Code (Loaded onto the Arduino UNO on the Robocone)
[25] – roughly 70 precent of the code is the referenced individual’s work */

#include <Servo.h>  //servo library
#include <Wire.h>
#include<SoftwareSerial.h>
SoftwareSerial XBee(2, 3);

Servo myservo;      // create servo object to control servo

// Map size 8*8
#define row 8
#define col 8

const int Echo = 11;  //digital pin 11 asigned for echo
const int Trig = 10;  //digital pin 10 asigned for trig

float duration, distance;   
int obstacle;

byte goalN = 63; // goal position on grid
byte openList[100]; // contains all the possible paths
byte closedList[100]; // contains the path taken
byte Path[100];
byte oLN=0, cLN=0;//the counters for the openList and closedList
byte curBotPos = 0 ; // holds current bot position
byte curBotDir = 1 ; // holds current bot facing direction(1 up  2 down 3 left  4 right)
byte curBotPos2;
byte curBotDir2;
char order = '0';

struct Node
{
  byte g, h, f;
  byte parent;
  byte index;
  byte gridNom;
};

struct Grid
{
  Node Map[row][col];
} PF ;


byte H(byte curR, byte curC, byte goalS)  // manhattan distance heauristics function
{
 byte rowg, colg;
 byte manhattan=0;

 
   rowg = (byte)goalS/8;
   colg = goalS%8;
   manhattan += (abs(curR - rowg) + abs(curC - colg));
   
  return manhattan;
}


byte G(byte curR, byte curC)  // returns the number of gride have been traverd
{
  byte gValue, parInd;
  byte rowg, colg;
  parInd = PF.Map[curR][curC].parent;
 
  rowg = (byte)parInd/8;
  colg = parInd%8;
  gValue = PF.Map[rowg][colg].g;
  
  return (gValue+1);
}

byte FV(byte curG, byte curH) // the total "cost" of the path taken; adds H and G values for each tile
{

  
 byte fValue; 
  
  fValue = curG + curH;
  return fValue;
}

void setup(){ // sets up the program, builds the map, prints the grid for representation purposes, and takes user input for the goal
 // Serial.flush();
  myservo.attach(12);  // attach servo on pin 12 to servo object
  Wire.begin();
  Serial.begin(9600);  //serial / USB port 
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT); 

  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
  XBee.begin(9600); 
  Serial.println(F("Starting XBee Comunication for Router "));
  
  //Serial.println("XBee Ready: Press Enter in the Serial ");
 // while  (Serial.available() == 0  )  {}
 //             Serial.write(XBee.read());
  
  //stop();

  myservo.write(90); //Put the ultrasonic in front of car
  
  buildMap();
  printGrid1();
  printGrid2();
  setGoal();
  
}

// checks if the goal tile has been found
void loop(){  
  
  if (!isGoal(curBotPos) && OLE)
  {
    _loop();                                      // the actual performance of the A* algorithm
  }
  else if (isGoal(curBotPos))
  {
    
    PathList();                                   // List the optimal path
    
    Serial.println(F("Path[i]"));
    for(byte i=0;i<PF.Map[closedList[cLN-1]/8][closedList[cLN-1]%8].g;i++) {
    Serial.println(Path[i]);
    }
   delay(1000);
    while (1){
      
      movement(curBotPos,curBotDir);
      curBotPos = curBotPos2;
      curBotDir = curBotDir2;
        
        if (!isGoal(curBotPos)){
          break;      
        }
        
        Serial.println(F("Goal Reached"));
        
        myservo.write(15);    // a little head dance the MONA does to show they've reached their goal
        delay(500);
        myservo.write(165);
        delay(500);
        myservo.write(15);
        delay(500);
        myservo.write(165);
        delay(500);
        myservo.write(15);
        delay(500);
        myservo.write(165);
        delay(500);
        myservo.write(90);

       delay(75000);
     }   
  }  
}

void _loop(){                 // performs the A* algorithm, "main" program

  possMov(curBotPos);
  
  AddClosedList();
  
  printGrid2();
  
}

void buildMap() // builds the 8x8 map grid
{
  byte gridIn = 0;
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    PF.Map[i][j].gridNom = gridIn;
    PF.Map[i][j].index = 0;
    PF.Map[i][j].parent = 0;
    PF.Map[i][j].h = 0;
    PF.Map[i][j].g = 0;
    PF.Map[i][j].f = 0;
    
    gridIn++;    
   }
  }
}

void printGrid1()  // prints the grid, using indices 0 to 99 to represent the possible paths
{
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    Serial.print(PF.Map[i][j].gridNom);
    if (j != row-1)
    {
      if (PF.Map[i][j].gridNom < row-1)
      {
        Serial.print(F("  | "));
      }
      else
    Serial.print(F(" | "));
    } 
   }
  
  if (i != row-1)
    {
      Serial.println();
    Serial.print(F("--------------------------------------"));
    Serial.println();
    }
  }
  Serial.println();
  Serial.println();
}

void printGrid2() // prints the grid, 0 - untravelled | 1 - travelled | 2 - obstacles | 3 - goal
{
  for (byte i = 0; i < row; i++)
  {
   for (byte j = 0; j < col; j++)
   {
    Serial.print(PF.Map[i][j].index);
    if (j != row-1)
    {
    Serial.print(F(" | "));
    } 
   }
  
  if (i != row-1)
    {
      Serial.println();
    Serial.print(F("-------------------------------"));
    Serial.println();
    }
  }
  Serial.println();
  Serial.println();
}

void setGoal() // asks user for input to set the goal state/tile
{
  byte goal;
 // Serial.println(curBotPos);
 // Serial.println(goal);
  Serial.println(F("Where do you want to place your goal state?"));
  Serial.println(F("Using the numbers displayaed in the earlier grid, enter a number to intialize as your goal state."));
  delay(250);
  //Serial.println(F("*"));
  //XBee.write('*');
 
 // while (Serial.available() == 0)
 //{
     
     
     //  goal = XBee.read();
     goal = 63;

  //}
  for (byte i = 0; i < row; i++)
  {
    for (byte k = 0; k < col; k++)
    {
      if (PF.Map[i][k].gridNom == goal)
      {
        
        PF.Map[i][k].index = 3;
        goalN = PF.Map[i][k].gridNom;
      }
      else if (PF.Map[i][k].gridNom == 0)  /// initial start point 
      {
        PF.Map[i][k].index = 1;
        curBotPos = PF.Map[i][k].gridNom;
      }
      else if (PF.Map[i][k].gridNom == 16 || PF.Map[i][k].gridNom == 17 || PF.Map[i][k].gridNom == 18 || PF.Map[i][k].gridNom == 20 || PF.Map[i][k].gridNom == 34 || PF.Map[i][k].gridNom == 43 || PF.Map[i][k].gridNom == 44 || PF.Map[i][k].gridNom == 61 )  //obsticles pre-written by the user
      {
        PF.Map[i][k].index = 2;        // initial wall
      }
      else
      PF.Map[i][k].index = 0;          // initial free space
    }
  }
  printGrid2();
}
void possMov(byte gridNom) // checks the possible moves depending on the location of the current tile the bot is on
{
  byte rowp = (byte) gridNom / 8;
  byte colp = gridNom % 8;
  if (gridNom == 0) // checks the corner tiles | 2 possible moves
  {
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }

    if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 8);
    }
  }
  else if (gridNom == 7)
  {
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }

    if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 8);
    }
  }
  else if (gridNom == 56)
  {
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }

    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 8);
    }
  }
  else if (gridNom == 63)
  {
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 8);
    }

    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 1);
    }   
  }
  else if (gridNom > 0 && gridNom < 7) // checks the tiles on the outermost edges of the map | 3 possible moves
  {
   if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    } 
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 8);
    }    
  }
  else if (gridNom%8==0)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 8);
    } 
    if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 8);
    }
  }
  else if (gridNom%8==7)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 8);
    } 
    if (PF.Map[rowp][colp- 1].index != 1 && PF.Map[rowp][colp- 1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 8);
    }
  }
  else if (gridNom > 56 && gridNom < 63)
  {
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 8);
    } 
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
    }
  }
  else { // checks the remaining tiles | 4 possible moves
    if (PF.Map[rowp-1][colp].index != 1 && PF.Map[rowp-1][colp].index != 2 && (!alreadyOnOL(rowp-1,colp)))
    {
      PF.Map[rowp-1][colp].parent = gridNom;
      AddOpenList(gridNom - 8);
    } 
    if (PF.Map[rowp][colp-1].index != 1 && PF.Map[rowp][colp-1].index != 2 && (!alreadyOnOL(rowp,colp-1)))
    {
      PF.Map[rowp][colp-1].parent = gridNom;
      AddOpenList(gridNom - 1);
    }
       if (PF.Map[rowp][colp+1].index != 1 && PF.Map[rowp][colp+1].index != 2 && (!alreadyOnOL(rowp,colp+1)))
    {
      PF.Map[rowp][colp+1].parent = gridNom;
      AddOpenList(gridNom + 1);
  }
     if (PF.Map[rowp+1][colp].index != 1 && PF.Map[rowp+1][colp].index != 2 && (!alreadyOnOL(rowp+1,colp)))
    {
      PF.Map[rowp+1][colp].parent = gridNom;
      AddOpenList(gridNom + 8);
  }
}

}

void AddOpenList(byte aol) // adds the potential possible moves to the openList
{
  
  openList[oLN++] = aol;
  heuristics(aol);
}

void heuristics(byte curIn) // calculates the "cost" of the tile
{
  byte hH, gH, fH;
  byte rowh = (byte) curIn / 8;
  byte colh = curIn % 8;

  hH = H(rowh, colh, goalN);
  PF.Map[rowh][colh].h = hH;
  gH = G(rowh, colh);
  PF.Map[rowh][colh].g = gH;
  fH = FV(hH,gH);
  PF.Map[rowh][colh].f = fH;
}

byte getNextFI() // returns the best heuristics value restricted by the current path the bot is taking
{
  byte rowf;
  byte colf;
  byte lowestF;
  byte lowest = openList[0];
  rowf = (byte) lowest / 8;
  colf = lowest % 8;
  lowestF = PF.Map[rowf][colf].f;
  
  for (byte i = 0; i < oLN; i++)
  {
    rowf = (byte) openList[i] / 8;
    colf = openList[i] % 8;
    
    if (PF.Map[rowf][colf].f <= lowestF) 
    {
      lowestF = PF.Map[rowf][colf].f;
      lowest = rowf*8 + colf;
    }
  }
  
  return lowest;
}

void AddClosedList() // adds the "best" tile to the closedList
{
  byte low = getNextFI(); 
  byte rowa, cola;

  closedList[cLN++] = low;
  rowa = (byte)low/8;
  cola = low%8;
  PF.Map[rowa][cola].index = 1;
  curBotPos = low;
  removeFOL(low); 
}

void PathList()  // List the optimal path
{
    for(byte i=1;i<PF.Map[closedList[cLN-1]/8][closedList[cLN-1]%8].g+1;i++){
      for(byte j=0;j<cLN;j++){
        if(PF.Map[closedList[j]/8][closedList[j]%8].g == i){
          Path[i-1]=closedList[j];
        }
      }
    }
}

void removeFOL(byte rfol) // removes previous potential paths from the openList, in order to get the "best" current path
{

  for (byte i = 0; i < oLN-56; i++)
  {
    if (openList[i] == rfol)
    {
      openList[i] = openList[i+1];
    }
    else
      openList[i] = openList[i+1];
  }
    oLN=oLN-1;
}

bool OLE() // checks if the openList is empty
{
  if (oLN == 0)
  {
    return true;
  }
  else
  return false;
}

bool isGoal(byte ig) // checks if the goal has been reached
{
  if (ig == goalN)
  {
    return true; 
  }
  else
  return false;
}

bool alreadyOnOL(byte rowaol, byte colaol) // checks if the tile is already on the openList
{
  byte indexol;
  bool on = false;

  indexol = rowaol*8 + colaol;
  for (byte i = 0; i < oLN; i++)
  {
    if (openList[i] == indexol)
    {
      on = true;
    }
  }
  
  return on;
}

byte movement(byte curBotPos,byte curBotDir) {
  
  curBotPos = PF.Map[Path[0]/8][Path[0]%8].parent;
  Serial.print(F("curBotPos_beforemovement:  "));
  Serial.println (curBotPos);
  Serial.print(F("curBotDir_beforemovement:  "));
  Serial.println (curBotDir);
  
  byte rowm, colm, parm;
  byte i = 0;

  while(!isGoal(curBotPos)){
     
      rowm = Path[i]/8;
      colm = Path[i]%8;
  
      if(Path[i] == PF.Map[rowm][colm].parent + 8 && curBotDir == 1){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 1){
        left();
        curBotDir = 3;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 1){
        right();
        curBotDir = 4;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent - 8 && curBotDir == 1){
        right();
        right();
        curBotDir = 2;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;

      }

      else if(Path[i] == PF.Map[rowm][colm].parent - 8 && curBotDir == 2){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 2){
        right();
        curBotDir = 3;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;

      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 2){
        left();
        curBotDir = 4;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent + 8 && curBotDir == 2){
        right();
        right();
        curBotDir = 1;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 3){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 8 && curBotDir == 3){
        right();
        curBotDir = 1;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 8 && curBotDir == 3){
        left();
        curBotDir = 2;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 3){
        right();
        right();
        curBotDir = 4;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;      
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 1 && curBotDir == 4){
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent - 8 && curBotDir == 4){
        right();
        curBotDir = 2;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
      else if(Path[i] == PF.Map[rowm][colm].parent + 8 && curBotDir == 4){
        left();
        curBotDir = 1;        
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
        else if(Path[i] == PF.Map[rowm][colm].parent + 1 && curBotDir == 4){
        right();
        right();
        curBotDir = 3;
        if (check_obstacle() == 1) {
           rePathPlan(curBotPos,curBotDir);
           break;
        }
        forward();
        curBotPos = Path[i];
        i++;
      }
    }
  Serial.print(F("RobotPosition in re-path:  "));
  Serial.println (curBotPos); 
  Serial.print(F("RobotDirection in re-path: "));
  Serial.println (curBotDir);
  curBotPos2 = curBotPos;
  curBotDir2 = curBotDir;
  return curBotPos2,curBotDir2;
  
  }

void rePathPlan(byte curBotPos,byte curBotDir) // re-design the path if encounter obstacles
{
 
  for (byte i = 0; i < 64; i++){

    if(PF.Map[i/8][i%8].index == 1){
      PF.Map[i/8][i%8].index = 0;
    }
    PF.Map[i/8][i%8].g = 0;
    PF.Map[i/8][i%8].h = 0;
    PF.Map[i/8][i%8].f = 0;
    PF.Map[i/8][i%8].parent = 0;
  }
   PF.Map[curBotPos/8][curBotPos%8].index = 1;
   PF.Map[goalN/8][goalN%8].index = 3;
  if(curBotDir == 1){
   PF.Map[(curBotPos + 8)/8][(curBotPos + 8)%8].index = 2;
  }
  else if(curBotDir == 2){
   PF.Map[(curBotPos - 8)/8][(curBotPos - 8)%8].index = 2;
  }
  else if(curBotDir == 3){
   PF.Map[(curBotPos + 1)/8][(curBotPos + 1)%8].index = 2;
  }
  else if(curBotDir == 4){
   PF.Map[(curBotPos - 1)/8][(curBotPos - 1)%8].index = 2;
  }
  
  oLN=0;
  cLN=0;

  for (byte i = 0; i<100; i++){
    openList[i] = 0; // contains all the possible paths
    closedList[i] = 0; // contains the path taken
    Path[i] = 0 ;   
  }
 Serial.print(F("curBotPos in re-path: "));
 Serial.println(curBotPos);
 Serial.print(F("curBotDir in re-path: "));
 Serial.println(curBotDir);
 printGrid2();
 
}
//Ultrasonic distance measurement Sub function
int check_obstacle(){
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW); 

  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58;   

// Measure the response from the echo pin

// Determin distance from duration
// Use 343 metres per second as speed of sound
   if(Fdistance <= 10){
    obstacle = 1;
      Serial.println(F("Obstacle detected!"));
      Serial.println();
      Serial.println(Fdistance);
      Serial.println();
   }
   else{
    obstacle = 0;
   }
   return obstacle;
}
// Function for forward
void forward(){ 
  
  order = '1';
  Serial.println(F("forward"));
  Wire.beginTransmission(9);
  Wire.write(order); 
  Wire.endTransmission(); 
  delay(5000);
  stop();
}


void back() {
  
  order = '2';
  Serial.println(F("Back"));
  Wire.beginTransmission(9);
  Wire.write(order); 
  Wire.endTransmission();
  delay(5000); 
  stop();

}

void left() {

  order = '3';
  Serial.println(F("Left"));
  Wire.beginTransmission(9);
  Wire.write(order); 
  Wire.endTransmission(); 
  delay(5000);
  stop();
}

void right() {

  order = '4';
  Serial.println(F("Right"));
  Wire.beginTransmission(9);
  Wire.write(order); 
  Wire.endTransmission(); 
  delay(5000);
  stop();
}

void stop() {

  Serial.println(F("Stop!"));
  delay(1000); 
  }
