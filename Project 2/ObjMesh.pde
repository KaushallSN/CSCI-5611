// Created for CSCI 5611 by Dan Shervheim

class ObjMesh
{
  private PShape _shape;
  
  public PVector position;
  public PVector rotation;
  public float scale;
  
  public ObjMesh(String meshPath)
  {
    _shape = loadShape(meshPath);
    
    position = new PVector(0, 0, -5.0);
    rotation = new PVector(0, 0, 180);
    scale = 10.0f;
  }
  
  // Draws the mesh at its own transform.
  public void draw()
  {
    drawWithTransform(this.position, this.rotation, this.scale);
  }
  
  // Draws the mesh multiple times at the given transforms. Note that each array must be the same length.
  public void drawInstanced(PVector[] positions, PVector[] rotations, float[] scales)
  {
    if (positions.length != rotations.length) {
      return;
    }
    if (positions.length != scales.length) {
      return;
    }
    
    for (int i = 0; i < positions.length; i++) {
      drawWithTransform(positions[i], rotations[i], scales[i]);
    }
  }
  
  private void drawWithTransform(PVector newPosition, PVector newRotation, float newScale)
  {
    pushMatrix();
    translate(newPosition.x, newPosition.y, newPosition.z);
    rotateX(newRotation.x * 3.1415926536 / 180.0);
    rotateY(newRotation.y * 3.1415926536 / 180.0);
    rotateZ(newRotation.z * 3.1415926536 / 180.0);
    scale(newScale, newScale, newScale);
    shape(_shape);
    popMatrix();
  }
}
