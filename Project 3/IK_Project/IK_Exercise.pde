//CCD and FABRIK Inverse Kinematics

// Sim Parameters
int dim_x = 1280;
int dim_y = 720;
PImage wheel_sprite, body_sprite, obstacle_sprite, bg; 

// Robot body
float body_len = 200; 
float body_width = 170;
float wheel_radius = 120;
Vec2 vel = new Vec2(0,0);
float speed = 30;

// Obstacle Stuff
Vec2 obs_pos;
float obs_radius = 20;
boolean left_grabbed = false;
boolean right_grabbed = false;


//Left Root
Vec2 left_root = new Vec2(dim_x/2,dim_y/2);

//Left Upper Arm
float left_arm_length0 = 120; 
float left_a0 = 3.14; //Shoulder joint

//Left Lower Arm
float left_arm_length1 = 100;
float left_a1 = 0; //Elbow joint

//Left Hand
float left_arm_length2 = 50;
float left_a2 = 0; //Wrist joint

//Left  Finger
float left_arm_length3 = 20;
float left_a3 = 0; //Wrist joint

float accCap = 0.01;

Vec2 left_start_l1,left_start_l2, left_start_l3, left_endPoint, goal;

//Right Root
Vec2 right_root = new Vec2(left_root.x + body_width, left_root.y);
float lengths[] = new float[5];
Vec2 p[] = new Vec2[6];
Vec2 p_prime[] = new Vec2[6];
Vec2 p_double_prime[] = new Vec2[6];

void setup(){
  
  size(1280,720, P3D);
  surface.setTitle("Project 3");
  
  body_sprite = loadImage("Body.png");
  wheel_sprite = loadImage("wheel.png");
  obstacle_sprite = loadImage("obstacle.png");
  bg = loadImage("bg.png");
  
  obs_pos = new Vec2(left_root.x + body_width/2, left_root.y + body_len/2);
  goal = obs_pos;
  
  for(int i = 0; i < 5; i++){
    lengths[i] = 50;
  }
  for(int i = 0; i < 6; i++){
    p[i] = new Vec2(right_root.x + i*lengths[0], right_root.y);
  }
}

void drawTexturedCircle(PVector position, float cRadius, PImage texture){
  textureMode(NORMAL);
  beginShape();
  texture(texture);
  for (int i = 0; i<40; i++) {
      float stepFac = 2*PI*(float(i)/40);
      vertex(position.x+cRadius/2 * sin(stepFac),position.y+cRadius/2 * cos(stepFac),(sin(stepFac)+1)/2,(cos(stepFac)+1)/2);
  }
  endShape(CLOSE);
}

void fabrik(){
  Vec2 dir;
  float a[] = new float[5];
  
  if(p[5].minus(goal).length() < 0.5) return;
  p_prime[5] = goal;
  for(int i = 5; i > 0; i--){
    dir = p[i-1].minus(p_prime[i]).normalized();
    p_prime[i-1] = p_prime[i].plus(dir.times(lengths[i-1]));
  }
  
  p_double_prime[0] = right_root;
  for(int i = 1; i < 6; i++){
    dir = p_prime[i].minus(p_double_prime[i-1]).normalized();
    p_double_prime[i] = p_double_prime[i-1].plus(dir.times(lengths[i-1]));
    
    
  }
  
  // Angles, in case we needed them
  a[0] = acos(dot(p_double_prime[1].minus(p_double_prime[0]).normalized(), new Vec2(1,0)));
  for(int i = 1; i < 5; i++){
    a[i] = acos(dot(p_double_prime[i].minus(p_double_prime[i-1]).normalized(), p_double_prime[i+1].minus(p_double_prime[i]).normalized()));
  }
  
  
  for(int i = 0; i < 6; i++){
    p[i] = p_double_prime[i];
  }
  
  // Attempts at angle constraints; they didn't work out
  /*
  for(int i = 0; i < 6; i++){
    if(i == 5){
      p[i] = p_double_prime[i];
    }
    else if(abs(a[i]) <= 3.14/2){
      p[i] = p_double_prime[i];
    }
  }
  */
  /*
  for(int i = 0; i < 6; i++){
    if(i == 0 || i == 5){
      p[i] = p_double_prime[i];
    }
    else if(abs(a[i-1]) > 3.14/2){
      while(abs(a[i-1]) > 3.14/2){
         dir = p[i].minus(p_double_prime[i]).normalized();
         p_double_prime[i] = p_double_prime[i].plus(dir);
         
         dir = p_double_prime[i].minus(p_double_prime[i-1]).normalized();
         p_double_prime[i] = p_double_prime[i-1].plus(dir.times(lengths[i-1]));
         
         a[i-1] = acos(dot(p_double_prime[i].minus(p_double_prime[i-1]).normalized(), p_double_prime[i+1].minus(p_double_prime[i]).normalized()));
      }
    }
  }
  */
}

void solve(){
  
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  //Update arm joint
  startToGoal = goal.minus(left_start_l3);
  startToEndEffector = left_endPoint.minus(left_start_l3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if(angleDiff > accCap) angleDiff = accCap;
  if (cross(startToGoal,startToEndEffector) < 0)
    left_a3 += angleDiff;
  else
    left_a3 -= angleDiff;
  
  if(angleDiff > accCap) angleDiff = accCap;
  if(left_a3 > 3.14/4){
    left_a3 = (3.14/4);
  }
  else if(left_a3 < -3.14/4){
    left_a3 = (-3.14/4);
  }
  fk();
  
  
  //Update wrist joint
  startToGoal = goal.minus(left_start_l2);
  startToEndEffector = left_endPoint.minus(left_start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if(angleDiff > accCap) angleDiff = accCap;
  if (cross(startToGoal,startToEndEffector) < 0)
    left_a2 += angleDiff;
  else
    left_a2 -= angleDiff;
  if(angleDiff > accCap) angleDiff = accCap;
  /*TODO: Wrist joint limits here*/
  if(left_a2 > 3.14/2){
    left_a2 = (3.14/2);
  }
  else if(left_a2 < -3.14/2){
    left_a2 = (-3.14/2);
  }
  fk();
  
  
  
  //Update elbow joint
  startToGoal = goal.minus(left_start_l1);
  startToEndEffector = left_endPoint.minus(left_start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if(angleDiff > accCap) angleDiff = accCap;
  if (cross(startToGoal,startToEndEffector) < 0)
    left_a1 += angleDiff;
  else
    left_a1 -= angleDiff;
  fk();
  
  
  //Update shoulder joint
  startToGoal = goal.minus(left_root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = left_endPoint.minus(left_root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if(angleDiff > accCap) angleDiff = accCap;
  if (cross(startToGoal,startToEndEffector) < 0)
    left_a0 += angleDiff;
  else
    left_a0 -= angleDiff;
  
  if(angleDiff > accCap) angleDiff = accCap;
  if(left_a0 > 3.14*1.5){
    left_a0 = (3.14*1.5);
  }
  else if(left_a0 < (3.14/2)){
    left_a0 = (3.14/2);
  }
  
  fk();
}

void fk(){
  left_start_l1 = new Vec2(cos(left_a0)*left_arm_length0,sin(left_a0)*left_arm_length0).plus(left_root);
  left_start_l2 = new Vec2(cos(left_a0+left_a1)*left_arm_length1,sin(left_a0+left_a1)*left_arm_length1).plus(left_start_l1);
  left_start_l3 = new Vec2(cos(left_a0+left_a1+left_a2)*left_arm_length2,sin(left_a0+left_a1+left_a2)*left_arm_length2).plus(left_start_l2);
  left_endPoint = new Vec2(cos(left_a0+left_a1+left_a2+left_a3)*left_arm_length3,sin(left_a0+left_a1+left_a2+left_a3)*left_arm_length3).plus(left_start_l3);
}

void update(float dt){
  keyPressed();
  fk();
  if((obs_pos.x < left_root.x + body_width/2  || left_grabbed) && !right_grabbed){
    solve();
    if((left_endPoint.minus(obs_pos).length() < obs_radius)){
      obs_pos = left_endPoint;
      goal = new Vec2(left_root.x + body_width/2, left_root.y + body_len/2);
      left_grabbed = true;
      // We've brought the object to our body
      if(goal.minus(obs_pos).length() < 0.9){
        obs_pos = new Vec2(random(dim_x), random(dim_y));
        goal = obs_pos;
        left_grabbed = false;
      }
    }
    else{
      goal = obs_pos;
      left_grabbed = false;
    }
  }
  else if((obs_pos.x > left_root.x + body_width/2 + 1  || right_grabbed) && !left_grabbed){
    fabrik();
    if((p[5].minus(obs_pos).length() < obs_radius)){
      obs_pos = p[5];
      Vec2 dir = new Vec2(left_root.x + body_width/2, left_root.y + body_len/2);
      dir = dir.minus(obs_pos);
      if(dir.length() > 1){
        dir = dir.normalized();
      } 
      goal = obs_pos.plus(dir);
      right_grabbed = true;
      // We've brought the object to our body
      if(goal.minus(p[5]).length() < 0.9){
        obs_pos = new Vec2(random(dim_x), random(dim_y));
        goal = obs_pos;
        right_grabbed = false;
      }
    }
    else{
      Vec2 dir = obs_pos.minus(p[5]).normalized();
      goal = p[5].plus(dir);
      right_grabbed = false;
    }
  }
  
  // Update Velocity
  left_root = new Vec2(left_root.x + vel.x * dt, left_root.y + vel.y * dt);
  right_root = new Vec2(right_root.x + vel.x * dt, right_root.y + vel.y * dt);
  for(int i = 0; i < 6; i++){
     p[i] = new Vec2(p[i].x + vel.x * dt, p[i].y + vel.y * dt); 
  }
  
}


float armW = 20;
void draw(){
  
  update(1/frameRate);
  
  background(bg);
  
  // Body
  fill(0,100,200);
  pushMatrix();
  translate(left_root.x,left_root.y);
  
  beginShape();
  texture(body_sprite);
  // vertex( x, y, z, u, v) where u and v are the texture coordinates in pixels
  vertex(0, 0, 0, 0, 0);
  vertex(body_width, 0, 0, body_sprite.width, 0);
  vertex(body_width, body_len, 0, body_sprite.width, body_sprite.height);
  vertex(0, body_len, 0, 0, body_sprite.height);
  endShape();
  
  popMatrix();
  
  // Wheel
  fill(0,0,0);
  pushMatrix();
  translate(left_root.x + body_width/2,left_root.y + body_len + wheel_radius/2);
  
  beginShape();
  texture(wheel_sprite);
  // vertex( x, y, z, u, v) where u and v are the texture coordinates in pixels
  vertex(-wheel_radius, -wheel_radius, 0, 0, 0);
  vertex(wheel_radius, -wheel_radius, 0, wheel_sprite.width, 0);
  vertex(wheel_radius, wheel_radius, 0, wheel_sprite.width, wheel_sprite.height);
  vertex(-wheel_radius, wheel_radius, 0, 0, wheel_sprite.height);
  endShape();
  
  popMatrix();
  

  //Right Arm
  fill(0, 255, 0);
  for(int i = 0; i < 5; i++){
    pushMatrix();
    translate(p[i].x,p[i].y);
    if(cross(p[i+1].minus(p[i]).normalized(), new Vec2(1,0)) < 0){
      rotate(acos(dot(p[i+1].minus(p[i]).normalized(), new Vec2(1,0))));
    }
    else{
      rotate(-1 * acos(dot(p[i+1].minus(p[i]).normalized(), new Vec2(1,0))));
    }
    rect(0, -armW/2, lengths[i], armW);
    popMatrix();
  }

  // Left arm
  fill(205,133,63);
  pushMatrix();
  translate(left_root.x,left_root.y);
  rotate(left_a0);
  rect(0, -armW/2, left_arm_length0, armW);
  popMatrix();
  
  pushMatrix();
  translate(left_start_l1.x,left_start_l1.y);
  rotate(left_a0+left_a1);
  rect(0, -armW/2, left_arm_length1, armW);
  popMatrix();
  
  pushMatrix();
  translate(left_start_l2.x,left_start_l2.y);
  rotate(left_a0+left_a1+left_a2);
  rect(0, -armW/2, left_arm_length2, armW);
  popMatrix();
  
  pushMatrix();
  translate(left_start_l3.x,left_start_l3.y);
  rotate(left_a0+left_a1+left_a2+left_a3);
  rect(0, -armW/4, left_arm_length3, armW/2);
  popMatrix();
  
  // Obstacle 
  fill(255, 0, 255);
  pushMatrix();
  translate(obs_pos.x, obs_pos.y);
  beginShape();
  texture(obstacle_sprite);
  // vertex( x, y, z, u, v) where u and v are the texture coordinates in pixels
  vertex(-obs_radius, -obs_radius, 0, 0, 0);
  vertex(obs_radius, -obs_radius, 0, obstacle_sprite.width, 0);
  vertex(obs_radius, obs_radius, 0, obstacle_sprite.width, obstacle_sprite.height);
  vertex(-obs_radius, obs_radius, 0, 0, obstacle_sprite.height);
  endShape();
  
  popMatrix();
  
  
}

void keyPressed(){
  if(mousePressed){
     obs_pos = new Vec2(mouseX, mouseY); 
  }
  if(key == 'w'){
     vel = new Vec2(0,-1 * speed);
  }
  if(key == 'a'){
     vel = new Vec2(-1 * speed,0);
  }
  if(key == 's'){
     vel = new Vec2(0, speed);
  }
  if(key == 'd'){
     vel = new Vec2(speed,0);
  }
  if(key == 'z'){
     vel = new Vec2(0,0); 
  }
  
}


//-----------------
// Vector Library
//-----------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
