// Kaushall Senthil Nathan - 5551829 (senth033@umn.edu)
// Project 2

Camera camera;

// Boolean Variables to toggle difference aspects of simulation
boolean isAirF = true;
boolean isSpringF = true;
boolean isCollision = true;
boolean seeClothParticles = false;
boolean seeClothTexture = true;
boolean isGravity = true;
boolean displayBounds = true;
boolean tearing = false;

//Rendering Meshes
ObjMesh snowManMesh;
ObjMesh towerMesh;

//Cloth set up variables
int num_hori = 13;
int num_verti = 26;
Vec3 pos[][] = new Vec3[num_hori][num_verti];
Vec3 buffer_pos[][] = new Vec3[num_hori][num_verti]; //useful in midpoint integration
Vec3 vel[][] = new Vec3[num_hori][num_verti];
Vec3 vel_buffer[][] = new Vec3[num_hori][num_verti];
Vec3 force[][] = new Vec3[num_hori][num_verti];
float gravity = 0.5;
float restLen = 0.5;
float k = 100;
float kv = 20;
float mass = 1.0;
Vec3 e;
float l, v1, v2, springF;

// Tearing Variables
float threshold = 400;
boolean springPresentHori[][] = new boolean[num_hori-1][num_verti];
boolean springPresentVerti[][] = new boolean[num_hori][num_verti-1];

// Air Sim Variables
float drag_coef = 1;
float density = 1;
Vec3 air_vel = new Vec3(0, 0, -3);
Vec3 airF, normal, rel_vel;

// Obstracle Variables
int num_obstacles = 1;
Vec3 obstaclePos[] = new Vec3[num_obstacles];
float obstacleRadius[] = new float[num_obstacles];
Vec3 obstacleNormal;
float COR = 0.5;


void setup()
{
  size(1000, 800, P3D);
  
/*
  for(int i = 0; i < num_hori; i++){
     for(int j = 0; j < num_verti; j++){
       
     }
  }
*/

  for(int i = 0; i < num_hori-1; i++){
     for(int j = 0; j < num_verti; j++){
       springPresentHori[i][j] = true;
     }
  }
  
  for(int i = 0; i < num_hori; i++){
     for(int j = 0; j < num_verti-1; j++){
       springPresentVerti[i][j] = true;
     }
  }
  
  for(int i = 0; i < num_hori; i++){
     for(int j = 0; j < num_verti; j++){
       pos[i][j] = new Vec3(-3 + restLen*i, -47, -49 - restLen*j);
       vel[i][j] = new Vec3(0,0,0);
       force[i][j] = new Vec3(0,0,0);
     }
  }
  
  obstaclePos[0] = new Vec3(0,-42,-50);
  obstacleRadius[0] = 4.7;
  
  camera = new Camera();
  
  towerMesh = new ObjMesh("LargeSquareTowerBricks.obj");
  towerMesh.position = new PVector(0,10,-50.0);
  towerMesh.rotation = new PVector(0, 270, 180);  // OBJ coordinate system y is reversed from processing, so rotate 180 to flip it.
  towerMesh.scale = 12;
  
  
  snowManMesh = new ObjMesh("Snowman.obj");
  snowManMesh.position = new PVector(-0, -42,-50.0);
  snowManMesh.rotation = new PVector(0, 90, 180);  // OBJ coordinate system y is reversed from processing, so rotate 180 to flip it.
  snowManMesh.scale = 6;
  
}

void keyPressed()
{
  camera.HandleKeyPressed();
  if(key == '1'){ println("Toggling Air"); isAirF = !isAirF;}
  if(key == '2'){ println("Toggling Spring"); isSpringF = !isSpringF;}
  if(key == '3'){ println("Toggling Collison"); isCollision = !isCollision;}
  if(key == '4'){ println("Toggling Cloth Particles"); seeClothParticles = !seeClothParticles;}
  if(key == '5'){ println("Toggling Textures"); seeClothTexture = !seeClothTexture;}
  if(key == '6'){ println("Toggling Gravity"); isGravity = !isGravity;}
  if(key == '7'){ println("Toggling Obstacle Boundary"); displayBounds = !displayBounds;}
  if(key == '8'){ println("Toggling Tearing"); tearing = !tearing;}
  if(key == 'r'){ println("restting"); setup();} 
  if(key == 'z'){ println("Doubled air speeed"); air_vel = air_vel.times(2);}
  if(key == 'c'){ println("Halved air speeed"); air_vel = air_vel.times(0.5);}
}

void keyReleased()
{
  camera.HandleKeyReleased();
}

void canTear(){
  for(int i = 0; i < num_hori; i++){
     for(int j = 0; j < num_verti; j++){
       if(i != num_hori - 1){ //horizontal springs
           if( (force[i][j].minus(force[i+1][j])).length() > threshold ) springPresentHori[i][j] = false;
       }
       if(j != num_verti - 1 && j != 0){
           if( (force[i][j].minus(force[i][j+1])).length() > threshold ) springPresentVerti[i][j] = false; 
       }
     }
  }
}

void calculateForces(float dt){
  
  // Obstacle Forces
  if(isCollision){
    for(int k = 0; k < num_obstacles; k++){
        for(int i = 0; i < num_hori; i++){
          for(int j = 0; j < num_verti; j++){
            if(pos[i][j].distanceTo(obstaclePos[k]) < (obstacleRadius[k])){
              obstacleNormal = (pos[i][j].minus(obstaclePos[k])).normalized();
              pos[i][j] = obstaclePos[k].plus(obstacleNormal.times(obstacleRadius[k]*1.01));
              force[i][j].add(projAB(vel[i][j], obstacleNormal).times((1 + COR)*-1/dt));
            }
          
          }
        }
    }
  }
  
  
  if(isSpringF){
    //Spring Forces
    for(int i = 0; i < num_hori; i++){
      for(int j = 0; j < num_verti; j++){
        // horizontal forces
        if(i != num_hori - 1){
          if(springPresentHori[i][j]){
            e = pos[i+1][j].minus(pos[i][j]);
            l = sqrt(dot(e,e));
            e.normalize();
            v1 = dot(e, vel[i][j]);
            v2 = dot(e, vel[i+1][j]);
            springF = (-k * (restLen - l)) - (kv * (v1 - v2));
            force[i][j].add(e.times(springF));
            force[i+1][j].subtract(e.times(springF));
          }
        }
        //vertical forces
        if(j != num_verti - 1){
          if(springPresentVerti[i][j]){
            e = pos[i][j+1].minus(pos[i][j]);
            l = sqrt(dot(e,e));
            e.normalize();
            v1 = dot(e, vel[i][j]);
            v2 = dot(e, vel[i][j+1]);
            springF = -k * (restLen - l) - kv * (v1 - v2);
            force[i][j].add(e.times(springF));
            force[i][j+1].subtract(e.times(springF));
          }
        }
      }
    }
  }
  // Gravity
  if(isGravity){
    for(int i = 0; i < num_hori; i++){
      for(int j = 0; j < num_verti; j++){
        force[i][j].add(new Vec3(0, gravity, 0));
      }
    }
  }
  //Air Forces
  if(isAirF){
    for(int i = 0; i < num_hori-1; i++){
      for(int j = 0; j < num_verti-1; j++){
        if(!springPresentHori[i][j] || !springPresentVerti[i][j]) continue; // if cloth is torn, then no air forces
        rel_vel = vel[i][j].plus(vel[i+1][j].plus(vel[i][j+1].plus(vel[i+1][j+1])));
        rel_vel = (rel_vel.times(1/4)).minus(air_vel);
        normal = cross(pos[i][j].minus(pos[i][j+1]), pos[i+1][j+1].minus(pos[i][j+1]));
        airF = normal.times((rel_vel.length() * dot(rel_vel, normal))/ (normal.length()));
        airF = airF.times(-0.5 * drag_coef * density);
        force[i][j] = force[i][j].plus(airF.times(1/4.0)); 
        force[i][j+1].add(airF.times(1/4.0));
        force[i+1][j+1].add(airF.times(1/4.0));
        force[i+1][j].add(airF.times(1/4.0));
      }
    }
  }
  
  if(tearing){
    canTear();
  }
  
  return; 
}

void resetForces(){
  for(int i = 0; i < num_hori; i++){
    for(int j = 0; j < num_verti; j++){
      force[i][j].mul(0);
    }
  }
}
  

void update(float dt){
  float half_dt = dt * 0.5;
  
  calculateForces(half_dt);
  
  for(int i = 0; i < num_hori; i++){
    for(int j = 0; j < num_verti; j++){
      buffer_pos[i][j] = pos[i][j];
      vel[i][j].add(force[i][j].times(half_dt/mass));
      if((i == 0 || i == num_hori -1) && j == 0){
        vel[i][j] = new Vec3(0,0,0);
      }
      pos[i][j].add(vel[i][j].times(half_dt));
    }
  }
  
  resetForces();
  
  calculateForces(dt);
  
  for(int i = 0; i < num_hori; i++){
    for(int j = 0; j < num_verti; j++){
      pos[i][j] = buffer_pos[i][j];
      vel[i][j].add(force[i][j].times(dt/mass));
      if((i == 0 || i == num_hori -1) && j == 0){
        vel[i][j] = new Vec3(0,0,0);
      }
      pos[i][j].add(vel[i][j].times(dt));
    }
  }
  
  resetForces();
  
}

void draw() {
  background(130, 130,255);
  directionalLight(200, 25 ,255, 1, 0, -1);
  directionalLight(255, 255 ,255, 0, 1, 0);
  update(1.0/(frameRate*2));
  camera.Update(1.0/frameRate);
  
  beginShape();
  fill(255);
  vertex(800, 10, 1000);
  vertex(800, 10, -1000);
  vertex(-800, 10, -1000);
  vertex(-800, 10, 1000);
  endShape();
  
  PImage cape = loadImage("Dandelion_painting.png");
  textureMode(NORMAL);
  beginShape();
  texture(cape);
  vertex(-100, 0, -200, 0, 1);
  vertex(100, 0, -200, 1, 1);
  vertex(100, -200, -200, 1, 0);
  vertex(-100, -200, -200, 0, 0);
  endShape();
  
  for(int i = 0; i < num_hori; i ++) {
     for(int j = 0; j < num_verti; j++) {

        if(i != num_hori - 1 && j != num_verti - 1){
          if(seeClothTexture && (springPresentHori[i][j] && springPresentVerti[i][j] && springPresentHori[i][j+1] && springPresentVerti[i+1][j])){
            /*
            if(i != 0){
               if(!springPresentHori[i-1][j]) continue; 
            }
            if(j != 0){
               if(!springPresentVerti[i][j-1]) continue; 
            }
            */
            beginShape();
            fill(255, 100, 100);
            vertex(pos[i][j].x,pos[i][j].y, pos[i][j].z, 0, 0);
            vertex(pos[i][j+1].x,pos[i][j+1].y, pos[i][j+1].z, 0, 1);
            vertex(pos[i+1][j+1].x,pos[i+1][j+1].y, pos[i+1][j+1].z, 1 ,1);
            vertex(pos[i+1][j].x,pos[i+1][j].y, pos[i+1][j].z, 1, 0);
            endShape();
          }
        }
        
        if(seeClothParticles){
          pushMatrix();
          translate(pos[i][j].x, pos[i][j].y, pos[i][j].z);
          sphere(2);
          popMatrix();
        }
     }
  }
  if(displayBounds){
    for(int i = 0; i < num_obstacles; i++){
      pushMatrix();
      translate(obstaclePos[i].x, obstaclePos[i].y, obstaclePos[i].z);
      sphere(obstacleRadius[i]);
      popMatrix();
    }
  }
  
  snowManMesh.draw();
  towerMesh.draw();
}
