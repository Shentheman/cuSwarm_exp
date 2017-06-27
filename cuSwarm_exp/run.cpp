#include "run.h"

/*************************************
***** OpenGL Callback Functions ******
*************************************/

void drawInterface(float window_width, float window_height) {

  // Draw edge only polygons (wire frame)
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glLineWidth(3.0f);

  // Color world boundaries based on whether the simulation is paused
  (paused) ? glColor4f(1.0f, 0.0f, 0.0f, 1.0f) : glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

  // Draw world boundaries
  float world_size_2 = p.world_size / 2.0f;
  glBegin(GL_POLYGON);
  glVertex3f(-world_size_2, -world_size_2, 0.0f);
  glVertex3f(-world_size_2, world_size_2, 0.0f);
  glVertex3f(world_size_2, world_size_2, 0.0f);
  glVertex3f(world_size_2, -world_size_2, 0.0f);
  glEnd();

  // Draw convex hull
  if (p.show_convex_hull) {
    glColor4f(0.9f, 0.9f, 0.1f, 0.7f);
    glBegin(GL_POLYGON);
    for (uint i = 0; i < data.ch.size(); i++) {
      glVertex3f(data.ch[i].x, data.ch[i].y, -0.1f);
    }
    glEnd();
  }

  // Draw filled polygons
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  // Draw explored grid cells on the GUI
  if (p.show_explored) {

    //for (uint i = 0; i < p.world_size * p.world_size; i++) {
      //// Get the world coordinates for this iteration
      //float world_x = -world_size_2 + (float)(floor(i / p.world_size));
      //float world_y = -world_size_2 + (float)(i % p.world_size);

      //// Now draw the grid cell if explored
      //float explored_color = 0.0f;
      //if (explored_grid[i] != 0) {
        //explored_color = fabsf((float)(explored_grid[i]) / p.max_explore);
        //// Color is based on obstacle/free space
        //if (explored_grid[i] > 0) {   // Free space
          //glColor4f(0.1f * explored_color, 0.3f * explored_color, 0.6f * explored_color, 0.5f);
        //}
        //else {              // Obstacle
          //// Lower bar for showing an obstacle cell as fully explored
          //explored_color = min(1.0f, explored_color * 4.0f);
          //glColor4f(0.6f * explored_color, 0.2f * explored_color, 0.2f * explored_color, 1.0f);
        //}
        //glBegin(GL_POLYGON);
        //glVertex3f(world_x, world_y, -0.2f);
        //glVertex3f(world_x + 1.0f, world_y, -0.2f);
        //glVertex3f(world_x + 1.0f, world_y + 1.0f, -0.2f);
        //glVertex3f(world_x, world_y + 1.0f, -0.2f);
        //glEnd();
      //}
    //}

    // Shen: to debug, draw all
    // Draw all the obstacles
    for (uint i = 0; i < p.num_obstacles; i++) {
      float4 obstacle = obstacles[i];
      for (uint xx = 0; xx < obstacle.z; xx++) {
        for (uint yy = 0; yy < obstacle.w; yy++) {
          float world_x = obstacle.x + xx;
          float world_y = obstacle.y + yy;
          float explored_color = 1.0f;
          glColor4f(0.6f * explored_color, 0.2f * explored_color, 0.2f * explored_color, 1.0f);
          glBegin(GL_POLYGON);
          glVertex3f(world_x, world_y, -0.2f);
          glVertex3f(world_x + 1.0f, world_y, -0.2f);
          glVertex3f(world_x + 1.0f, world_y + 1.0f, -0.2f);
          glVertex3f(world_x, world_y + 1.0f, -0.2f);
          glEnd();
        }
      }
    }
  }

  //x = int \in [-75, 74]
  //y = int \in [-75, 74]
  for (uint i = 0; i < p.world_size * p.world_size; i++) {
    float world_x = -world_size_2 + (float)(floor(i / p.world_size));
    float world_y = -world_size_2 + (float)(i % p.world_size);
    //std::cout<<"("<<world_x<<", "<<world_y<<")"<<std::endl;
  }
  //if (eucl2(world_x + 0.5f, world_y + 0.5f, 
        //positions[n].x, positions[n].y) <= p.range) {
 
 
  // Draw targets on the GUI
  for (uint i = 0; i < p.targets; i++) {

    // Get the explored grid index that this target corresponds to
    uint exp_ind = (uint)(((targets[i].x + ws_2) * p.world_size) + (targets[i].y + ws_2));

    // Saturate the target color based on explored value
    float saturation;
    // Target must be seen at least once to show
    (explored_grid[exp_ind] == 0) ? saturation = 0.0f : 
      saturation = fabsf(0.25f + (0.75f * 
            ((float)(explored_grid[exp_ind]) / p.max_explore)
            ));

    //Shen: For debug, draw the entire map
    saturation = 1.0f;

    // Change target color based on whether fully explored
    if (saturation < 1.0f) {
      // Purple (not fully explored)
      glColor4f(0.6f * saturation, 0.1f * saturation, 0.6f * saturation, 1.0f);
      // If first time target is seen, indicate so in target data field, 
      // and add to targets_seen count
      if (saturation > 0.0f && targets[i].z == 0) {
        targets[i].z = 1;
        data.targets_seen++;
      }
    }
    else {
      // Green (fully explored)
      glColor4f(0.1f * saturation, 0.8f * saturation, 0.1f * saturation, 1.0f);
      // If first time reaching fully explored status, indicate so in data
      // field and add to targets_explored count
      if (targets[i].z == 1) {
        targets[i].z = 2;
        int second = (int)((float)step_num / 60.0f);
        targets_by_second[second]++;
        data.targets_explored++;
      }
    }

    // Draw target
    float x = (float)(targets[i].x);
    float y = (float)(targets[i].y);
    glBegin(GL_POLYGON);
    glVertex3f(x, y, -0.1f);
    glVertex3f(x + 1.0f, y, -0.1f);
    glVertex3f(x + 1.0f, y + 1.0f, -0.1f);
    glVertex3f(x, y + 1.0f, -0.1f);
    glEnd();
  }

  // Set color to white for next GUI elements
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  // Clear model and projection matricies to draw interface
  glResetModelAndProjection();
  // Set fill mode for polygons
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // Draw time remining info
  stringstream ss;
  ss.precision(3);
  // Time remaining
  ss << "Time remaining:\n";
  ss << (int)((float)(p.step_limit - step_num) / 60.0f);
  ss << " seconds";
  drawText(-0.98f, -0.7f, (char*)ss.str().c_str(), 0.8f, 0.8f, 0.8f);
  
  // Draw user trust info
  drawText(0.735f, 0.9f, (char *)"Trust Level:", 0.8f, 0.8f, 0.8f);
  ss.str("");
  ss << data.user_trust;
  float x_pos;
  // Switch position of text value of trust level based on number of digits
  if (data.user_trust < 0 && data.user_trust > -10) {
    x_pos = 0.895f;
  }
  else if (data.user_trust == 10) {
    x_pos = 0.9055f;
  }
  else if (data.user_trust == -10) {
    x_pos = 0.871f;
  }
  else {
    x_pos = 0.93f;
  }
  drawText(x_pos, 0.8f, (char*)ss.str().c_str(), 0.8f, 0.8f, 0.8f);

  // Draw target label
  drawText(-0.98f, 0.9f, (char *)"Target information:", 0.8f, 0.8f, 0.8f);

  // Draw either text or visualization of performance (targets found) 
  // and swarm state properties (variance and area covered) 
  // based on parameter set
  if (p.show_info_graph) {

    // Draw running graph of targets found
    glBegin(GL_POLYGON);
    glVertex3f(-0.98f, 0.8f, 0.0f);
    glVertex3f(-0.50f, 0.8f, 0.0f);
    glVertex3f(-0.50f, 0.3f, 0.0f);
    glVertex3f(-0.98f, 0.3f, 0.0f);
    glEnd();

    // Get width of one second on the graph
    float width = 0.48f / 60.0f;
    // Get the number of seconds passed in simulation
    int seconds = (int)((float)step_num / 60.0f);
    // Variable to keep track of targets found in the last minute
    int targets_last_minute = 0;

    // Draw a line graph of the number of targets, 
    // heading variance, and area covered
    if (step_num > 0) {
      // Because the graph only shows the last 60 seconds, 
      // i is initialized to max(seconds - 60, 0)
      // We also separately keep track of i starting value 
      // for determining x coordinate
      int i = max(seconds - 60, 1);
      int start = i;
      for (i; i < seconds; i++) {
        // Add to targets found last minute
        targets_last_minute += targets_by_second[i - 1];

        // x coordinate of graph point (same for all variables)
        float x_start = -0.98f + ((i - start) * width);
        float x_end = x_start + width;

        // y coordinates for target point
        float y_start_target = 0.3f;
        float y_end_target = 0.3f + 
          (0.5f * ((float)targets_by_second[i - 1] / 10.0f));

        // y coordinate for swarm area
        float y_start_area = 0.3f + 
          (0.5f * fminf(area_by_second[i - 1] / 1500.0f, 1.0f));
        float y_end_area = 0.3f + 
          (0.5f * fminf(area_by_second[i] / 1500.0f, 1.0f));

        // Set color to dark green and draw the targets seen
        glColor4f(0.1f, 0.6f, 0.1f, 1.0f);
        glBegin(GL_LINES);
        // Don't use x_start here because we just want a vertical line with
        // height = to targets that second
        glVertex3f(x_start, y_start_target, 0.0f);
        glVertex3f(x_start, y_end_target, 0.0f);
        glEnd();

        // Set color to yellow and draw the targets seen
        glColor4f(0.7f, 0.7f, 0.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(x_start, y_start_area, 0.0f);
        glVertex3f(x_end, y_end_area, 0.0f);
        glEnd();
      }
    }

    // Traw textual information of targets
    ss.str("");
    ss << "Targets last 60s: " << targets_last_minute;
    drawText(-0.98f, 0.2f, (char*)ss.str().c_str(), 0.1f, 0.8f, 0.1f);
    ss.str("");
    ss << "Current swarm area: " << (int)area_by_second[seconds];
    drawText(-0.98f, 0.1f, (char*)ss.str().c_str(), 0.7f, 0.7f, 0.0f);
  }
  else {
    // Draw targets seen/found in text form
    ss.str("");
    ss << "Targets found: " << data.targets_explored;
    drawText(-0.98f, 0.8f, (char*)ss.str().c_str(), 0.8f, 0.8f, 0.8f);
  }

  // Set to fill in bars of each step
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  // Set color to gray
  glColor4f(1.0f, 1.0f, 1.0f, 0.75f);

  // Draw progress bar for time remaining
  glLineWidth(1.0f);
  float bar_x_end = -0.98f + (0.48f * ((float)(step_num) /
    (float)(p.step_limit)));
  // Outline
  glBegin(GL_POLYGON);
  glVertex3f(-0.98f, -0.82f, 0.0f);
  glVertex3f(-0.50f, -0.82f, 0.0f);
  glVertex3f(-0.50f, -0.95f, 0.0f);
  glVertex3f(-0.98f, -0.95f, 0.0f);
  glEnd();
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  // Filling
  glBegin(GL_POLYGON);
  glVertex3f(-0.98f, -0.82f, 0.0f);
  glVertex3f(bar_x_end, -0.82f, 0.0f);
  glVertex3f(bar_x_end, -0.95f, 0.0f);
  glVertex3f(-0.98f, -0.95f, 0.0f);
  glEnd();

  // Draw trust bar
  float bar_y_end = -0.12f + (0.83f * (data.user_trust / 10.0f));
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  // Outline
  glBegin(GL_POLYGON);
  glVertex3f(0.96f, -0.95f, 0.0f);
  glVertex3f(0.96f, 0.71f, 0.0f);
  glVertex3f(0.84f, 0.71f, 0.0f);
  glVertex3f(0.84f, -0.95f, 0.0f);
  glEnd();
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  // Filling
  // Set color of trust bar based on trust being positive or negative
  (data.user_trust < 0) ? glColor4f(0.8f, 0.0f, 0.0f, 1.0f) :
    glColor4f(0.0f, 0.8f, 0.0f, 1.0f);
  glBegin(GL_POLYGON);
  glVertex3f(0.96f, -0.12f, 0.0f);
  glVertex3f(0.96f, bar_y_end, 0.0f);
  glVertex3f(0.84f, bar_y_end, 0.0f);
  glVertex3f(0.84f, -0.12f, 0.0f);
  glEnd();

  // Set color to cyan for user inputs
  glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
  glLineWidth(2.0f);

  // Convert user-drawn line from screen to world coordinates
  float3 screen_start_point = make_float3(mouse_start_x, mouse_start_y, translate_z0);
  float3 screen_last_point = make_float3(mouse_last_x, mouse_last_y, translate_z0);
  float3 world_start_point = make_float3(0.0f, 0.0f, 0.0f);
  float3 world_last_point = make_float3(0.0f, 0.0f, 0.0f);
  screenToWorld(screen_start_point, &world_start_point);
  screenToWorld(screen_last_point, &world_last_point);
  // Draw the user-given direction
  glBegin(GL_LINES);
  glVertex3f(world_start_point.x, world_start_point.y, 0.0f);
  glVertex3f(world_last_point.x, world_last_point.y, 0.0f);
  glEnd();
}

void drawEllipse(float cx, float cy, float w, float h, bool fill)
{
  // Variables for incremental lines
  float theta = 2.0f * (float)(PI) / 100.0f;
  float c = cosf(theta);
  float s = sinf(theta);
  float t;
  float x = 1.0f;
  float y = 0.0f;

  // Set to filled or wire frame depending on fill parameter
  (fill) ? glPolygonMode(GL_FRONT_AND_BACK, GL_FILL) : glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // Draw the ellipse
  glBegin(GL_POLYGON);
  for (uint i = 0; i < 100; i++) {
    glVertex3f((w * x) + cx, (h * y) + cy, -0.2f);

    // Apply the rotation matrix
    t = x;
    x = c * x - s * y;
    y = s * t + c * y;
  }
  glEnd();
}

void drawText(float x, float y, char *string, GLfloat r, GLfloat g, GLfloat b)
{
  // Change to specified color
  glColor4f(r, g, b, 0.75f);

  // Draw text at the given point
  glTranslatef(x, y, 0.0f);
  glScalef(0.0003f, 0.00045f, 1.0f);
  glutStrokeString(GLUT_STROKE_ROMAN, (unsigned char*)string);

  // Clear model and projection matricies to draw interface
  glResetModelAndProjection();
}

void keyboard(unsigned char key, int x, int y)
{
  switch (key)
  {
  case 'w': {
    moveUp();
    break;
  }
  case 'a': {
    moveLeft();
    break;
  }
  case 's': {
    moveDown();
    break;
  }
  case 'd': {
    moveRight();
    break;
  }
  case ' ': {
    // Pause / resume the simulation if not checking user trust
    if (trust_verified) {
      paused = !paused;
    }
    break;
  }
  case 'z': { 
    // The following three ('z', 'x', 'c', are for current experiment training 
    // only, and will be removed later
    if (p.training) {
      p.range_r = 1.5f;
      p.range_f = 2.5f;
    }
    break;
  }
  case 'x': {
    if (p.training) {
      p.range_r = 2.5f;
      p.range_f = 3.5f;
    }
    break;
  }
  case 'c': {
    if (p.training) {
      p.range_r = 3.5f;
      p.range_f = 4.5f;
    }
    break;
  }
  case '1': {
    // Switch to rendezvous
    p.behavior = BEHAVIOR_RENDEZVOUS;
    break;
  }
  case '2': {
    // Switch to flocking
    p.behavior = BEHAVIOR_FLOCKING;
    break;
  }
  case '3': {
    // Switch to dispersion
    p.behavior = BEHAVIOR_DISPERSION;
    break;
  }
  case 27: { // Escape key
    exitSimulation();
    break;
  }
  case 13: { // Enter key
    trust_verified = true;
    paused = false;
    break;
  }
  }
}

// Handles directional arrow and function key presses
void keyboardSpecial(int key, int x, int y)
{
  switch (key) {
  case GLUT_KEY_UP: {
    moveUp();
    break;
  }
  case GLUT_KEY_LEFT: {
    moveLeft();
    break;
  }
  case GLUT_KEY_DOWN: {
    moveDown();
    break;
  }
  case GLUT_KEY_RIGHT: {
    moveRight();
    break;
  }
  }
}

void mouse(int button, int state, int x, int y)
{
  // If the button is pressed down (ignore subsequent buttons before the first
  // is released)
  if (state == GLUT_DOWN && mb == GLUT_NULL_BUTTON) {
    // Get mouse button
    mb = button;

    // Only process presses if unpaused
    if (!paused) {  
      // Primary buttons to give commands
     
      // 1. Left mb means trusted command, then command_trust = 1
      if (mb == GLUT_LEFT_BUTTON) {
        mouse_start_x = (float)(x);
        mouse_start_y = (float)(y);
        mouse_last_x = mouse_start_x;
        mouse_last_y = mouse_start_y;
      }

      // 2. Right mb means untrusted command, then command_trust = 2
      else if (mb == GLUT_RIGHT_BUTTON) {
        p.behavior = BEHAVIOR_RENDEZVOUS_TO_POINT;
        p.align_weight = 1.0f;
        // Set goal point to swarm's center of mass
        for (uint i = 0; i < p.num_robots; i++) {
          goal_point.x += positions[i].x;
          goal_point.y += positions[i].y;
        }
        goal_point.x /= p.num_robots;
        goal_point.y /= p.num_robots;

        // Log rendezvous command
        fprintf(output_f, "rendezvous\n");
      }
    }
    if (mb == GLUT_WHEEL_UP_PRESSED) {    // Scroll wheel forward to increase trust
      data.user_trust += 1;
      data.user_trust = min(data.user_trust, 10);
    }
    else if (mb == GLUT_WHEEL_DOWN_PRESSED) {   // Scroll wheel backward to decrease trust
      data.user_trust -= 1;
      data.user_trust = max(data.user_trust, -10);
    }
  }
  else if (state == GLUT_UP && mb == button) {  // If the button is released
    if (mb == GLUT_LEFT_BUTTON) { // Primary or seconday mouse button
      // If the simulation is paused, unpause it; 
      // else log the new user goal heading and log the information; 
      if (paused && trust_verified) {
        paused = false;
      }
      else if (!paused) {
        // Set behavior back to flocking
        p.behavior = BEHAVIOR_FLOCKING;
        // Reset command_trust (because command is finished)
        command_trust = 0;

        // Get the magnitude (length) of user-drawn vector
        float magnitude = eucl2(mouse_start_x, mouse_start_y, (float)x, (float)y);
        // Get the goal direction in radians
        goal_heading = atan2f((float)(y)-mouse_start_y, (float)(x)-mouse_start_x);
        // Transform this into a 2D unit vector (float3, but z not used)
        goal_vector = make_float3(cosf(goal_heading), -sinf(goal_heading), 0.0f);

        //// (752,653) => (1339,536)
        //std::cout<<"("<<mouse_start_x<<", "<<mouse_start_y<<") => ";
        //std::cout<<"("<<(float)x<<", "<<(float)y<<")"<<std::endl;
        //// magnitude = 635.542
        //std::cout<<"magnitude = "<<magnitude<<std::endl;
        //// goal_heading = -0.19674
        //std::cout<<"goal_heading = "<<goal_heading<<std::endl;
        //// goal_vector = (0.980,0.195,0)
        //std::cout<<"goal_vector = ("<<goal_vector.x<<", "<<goal_vector.y<<", "
          //<<goal_vector.z<<")"<<std::endl;

        // Log user command
        fprintf(output_f, "heading %f %f\n", goal_heading, magnitude);

        // Clear the user-drawn line data points
        mouse_start_x = 0;
        mouse_start_y = 0;
        mouse_last_x = 0;
        mouse_last_y = 0;
      }
    }
    else if (mb == GLUT_WHEEL_UP_RELEASED) { // Scroll wheel forward
      data.user_trust += 1;
      data.user_trust = min(data.user_trust, 10);
    }
    else if (mb == GLUT_WHEEL_DOWN_RELEASED) { // Scroll wheel backward
      data.user_trust -= 1;
      data.user_trust = max(data.user_trust, -10);
    }

    // If mouse_up event caused by scrolling while left mouse is down, reset 
    // mb to 0 (left click); else, reset to null
    if (mb == GLUT_WHEEL_UP_RELEASED || mb == GLUT_WHEEL_DOWN_RELEASED) {
      mb = GLUT_LEFT_BUTTON;
    }
    else {
      mb = GLUT_NULL_BUTTON;
    }
  }
}

void motion(int x, int y)
{
  // Draw the user heading line if the primary button is down and simulation 
  // is not paused
  if ((mb == GLUT_LEFT_BUTTON) && !paused) {
    mouse_last_x = (float)(x);
    mouse_last_y = (float)(y);
  }
}

void moveUp()
{
  translate_y0 += 1.0f;
}

void moveLeft()
{
  translate_x0 -= 1.0f;
}

void moveRight()
{
  translate_x0 += 1.0f;
}

void moveDown()
{
  translate_y0 -= 1.0f;
}

void glResetModelAndProjection()
{
  // Reinitialize OpenGL modelview and projection matricies
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

/****************************
***** OPENGL FUNCTIONS ******
****************************/

void initGL(int argc, char **argv)
{
  glutInit(&argc, argv);

  if (p.window_width == 0 || p.window_height == 0) {
    p.window_width = glutGet(GLUT_SCREEN_WIDTH);
    p.window_height = glutGet(GLUT_SCREEN_HEIGHT);
  }

  // Setup window
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(p.window_width, p.window_height);
  glutCreateWindow("CUDA Swarm Simulation");

  // If in Windows, start with the window maximized
#ifdef _WIN32
  // Get the handle for the current window
  HWND win_handle = FindWindow(0, "CUDA Swarm Simulation");
  if (!win_handle) {
    std::printf("!!! Failed FindWindow\n");
  }

  // Maximize the window for full screen simulation operation
  SetWindowLong(win_handle, GWL_STYLE, 
      (GetWindowLong(win_handle, GWL_STYLE) | WS_MAXIMIZE));
  ShowWindowAsync(win_handle, SW_SHOWMAXIMIZED);
#endif

  // Register OpenGL callbacks
  glutDisplayFunc(display);       // OpenGL display callback (looped)
  glutKeyboardFunc(keyboard);       // OpenGL keyboard callback
  glutSpecialFunc(keyboardSpecial);   // OpenGL keyboard special callback
  glutMouseFunc(mouse);         // OpenGL mouse callback
  glutMotionFunc(motion);         // OpenGL mouse motion callback
  glutPassiveMotionFunc(motion);      // OpenGL pass. mouse motion callback
  glutTimerFunc(1000, calculateFPS, 0); // Recalculate FPS every 1/2 second

  // GLEW initialization
  glewInit();
  if (!glewIsSupported("GL_VERSION_2_0")) {
    std::fprintf(stderr, "ERROR: Support for necessary OpenGL extensions missing.");
    fflush(stderr);
    exit(0);
  }
}

void createVBO(GLuint* vbo, struct cudaGraphicsResource **vbo_res,
  unsigned int vbo_res_flags, uint size) {
  // Create vertex buffer name
  glGenBuffers(1, vbo);

  /// Bind
  glBindBuffer(GL_ARRAY_BUFFER, *vbo);
  glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
  /// Unbind
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // Register VBO with CUDA
  int error = cudaGraphicsGLRegisterBuffer(vbo_res, *vbo, vbo_res_flags);
  if (error != 0) printf("ERROR createVBO 1 = %d\n",error);
}

void deleteVBO(GLuint *vbo, struct cudaGraphicsResource *vbo_res) {
  // unregister this buffer object with CUDA
  cudaGraphicsUnregisterResource(vbo_res);
  // Delete VBO
  glBindBuffer(1, *vbo);
  glDeleteBuffers(1, vbo);
  *vbo = 0;
}

static void display(void) {

  // Quit if not automated and last step reached; else take a sulation step
  if (step_num > p.step_limit) {
    exitSimulation();
  }
  else {
    step(0);
  }

  // Get the window dimensions
  int window_width = glutGet(GLUT_WINDOW_WIDTH);
  int window_height = glutGet(GLUT_WINDOW_HEIGHT);

  // Clear the display
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, window_width, window_height);

  // Draw interface elements
  drawInterface((float)(window_width), (float)(window_height));

  // Projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(75.0, (GLfloat)window_width / (GLfloat)window_height, 
      0.001, 500.0);

  // Change point size based on distance from camera
  glPointSize((float)(p.point_size));
  //with smooth, draw round point, otherwise, draw rectangle point
  glEnable(GL_POINT_SMOOTH);
  float quadratic[] = {0.05f, 0.0f, 0.001f};
  glPointParameterfvARB(GL_POINT_DISTANCE_ATTENUATION_ARB, quadratic);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Cap the translation values to +/- world size + 10
  translate_x0 = min(translate_x0, ws_2 + 10.0f);
  translate_x0 = max(translate_x0, -ws_2 - 10.0f);
  translate_y0 = min(translate_y0, ws_2 + 10.0f);
  translate_y0 = max(translate_y0, -ws_2 - 10.0f);
  translate_z0 = min(translate_z0, ws_2 * 2.0f);
  translate_z0 = max(translate_z0, 1.0f);

  // Apply view transforms
  glRotatef(rotate_x0, 1.0, 0.0, 0.0);
  glRotatef(rotate_y0, 0.0, 1.0, 0.0);
  glTranslatef(-translate_x0, -translate_y0, -translate_z0);

  // Closer things cover far things
  /// Shen TODO  to display obstacle more clearly
  //glEnable(GL_DEPTH_TEST);
  //glDepthFunc(GL_LESS);
  //glDepthMask(GL_TRUE);

  // Transparency
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  // 1. Draw agents from vbo
  /*
   * Here, we want to draw robot members and obstacle grids on the interface.
   * I. Use 2 VBOs, one for robot positions 
   * and the other one for obstacle positions.
   * Since both of them are GL_ARRAY_BUFFER, we have to create 2 VBOs
   * and bind both of them with GL_ARRAY_BUFFER in step().
   *
   * However, opengl does not allow us to bind two vbos with the same target.
   * https://www.opengl.org/discussion_boards/showthread.php/172530-Two-VBOs-at-once?p=1209767&viewfull=1#post1209767
   * https://stackoverflow.com/a/21540956
   * "You can use >=2 buffers to draw from, 
   * but only if they have different targets.
   * So here, if we bind the 2 VBOs with GL_ARRAY_BUFFER two times,
   * only the most recent one remains in effect.
   *
   * Here's a quote from OpenGL core 4.0 spec.:
   * BindBuffer may also be used to bind an existing buffer object. 
   * (1) If the bind is successful no change is made to the state of 
   * the newly bound buffer object, 
   * and any previous binding to target is broken.
   * (2) In the case of indexed data, then the indices are bound to 
   * target GL_ELEMENT_ARRAY_BUFFER 
   * while the vertices (and their attributes) are bound to GL_ARRAY_BUFFER; 
   * two different targets and so two different VBOs are okay.
   *
   * II. Combine all the vertex and attribute data into a single VBO.
   */
  /// We need to store 2 arrays in the same VBO
  /// (1) Draw each explored obstacle grid as a point
  /// (2) Draw each agent as a point
  /// vbo_name (GLuint) is the link to cuda_vbo_resource_swarm
  glBindBuffer(GL_ARRAY_BUFFER, vbo_name);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  // array = [x (float), y (float), z (float), c (4 unsigned byte RGBA), ...]
  // 4 in a pair for a vertex, so stride = 16 bytes
  // position starts from 0 bytes while color starts from 12 bytes
  glVertexPointer(3, GL_FLOAT, 16, 0);
  /// arg1 = the number of components per color (RGBAlpha)
  /// arg2 = the data type of each color component
  /// arg3 = stride = the byte offset between consecutive colors in the array.
  ///   If stride is 0, the colors are understood to be tightly packed in the array
  /// arg4 = a pointer to the 1st component of the 1st color in the array
  glColorPointer(4, GL_UNSIGNED_BYTE, 16, (GLvoid*)12);
  /// TODO: Some points are missing. But the VBO should have all the points
  /// from the past because the counter keeps increasing.
  glDrawArrays(GL_POINTS, 0, p.num_robots);
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  glBindBuffer(GL_ARRAY_BUFFER, 0);


  //!!!!!!!!!!!!!!!!!!???????????????????????????????
  int counter = 0;
  for (uint i = 0; i < p.num_robots*NUM_ANGLE_RAY_TRACE; i ++) {
    if (positions_obs_from_cuda[i].z == GRID_EXPLORED_OBS) {
      counter ++;
      uint index = (uint)((positions_obs_from_cuda[i].x+p.world_size/2.0f)
          +(uint)(floor(positions_obs_from_cuda[i].y+p.world_size/2.0f)
          *p.world_size));
      //printf("positions_obs_from_cuda[%d]=(%f,%f,%f,%f)\n", index, 
          //positions_obs_from_cuda[i].x, positions_obs_from_cuda[i].y,
          //positions_obs_from_cuda[i].z, positions_obs_from_cuda[i].w);
      assert(index >=0 && index < p.world_size*p.world_size);
      positions_obs[index] = positions_obs_from_cuda[i];
      //printf("obs at pos %d = (%f,%f,%f,%f)\n", 
          //i, positions_obs_from_cuda[i].x, positions_obs_from_cuda[i].y, 
          //positions_obs_from_cuda[i].z, positions_obs_from_cuda[i].w);
    }
  }
  printf("size = %d\n", counter);
  for (uint i = 0; i < p.world_size*p.world_size; i++) {
    std::cout<<positions_obs[i].x<<", "<<positions_obs[i].y<<
      ", "<<positions_obs[i].z<<", "<<positions_obs[i].w<<std::endl;
  }


  /// array = [x (float), y (float), c (4 unsigned byte RGBA), ...]
  std::vector<float> positions_obs_tmp(counter*3);
  if (counter > 0) {
    for (uint i = 0; i < p.world_size*p.world_size; i++) {
      if (positions_obs[i].z == GRID_EXPLORED_OBS) {
        positions_obs_tmp.emplace_back(positions_obs[i].x);
        positions_obs_tmp.emplace_back(positions_obs[i].y);
        positions_obs_tmp.emplace_back(positions_obs[i].w);
      }
    }
    for (uint i = 0; i < counter; i++) {
      std::cout<<positions_obs_tmp[i]<<", ";
    }

    glEnableClientState(GL_VERTEX_ARRAY);
    //glEnableClientState(GL_COLOR_ARRAY);
    glPointSize(100);
    // array = [x (float), y (float), c (4 unsigned byte RGBA), ...]
    // 3 in a pair for a vertex, so stride = 12 bytes
    // position starts from 0 bytes while color starts from 12 bytes
    glVertexPointer(2, GL_FLOAT, 12, &positions_obs_tmp[0]);
    /// arg1 = the number of components per color (RGBAlpha)
    /// arg2 = the data type of each color component
    /// arg3 = stride = the byte offset between consecutive colors in the array.
    /// arg4 = a pointer to the 1st component of the 1st color in the array
    //glColorPointer(4, GL_UNSIGNED_BYTE, 12, &positions_obs_tmp[0]+8);
    glDrawArrays(GL_POINTS, 0, counter);
    //glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
  }

  // 2. Draw robot data
  for (uint i = 0; i < p.num_robots; i++) {

    // Orientation lines
    if ((p.show_leaders && modes[i] == MODE_LEADER) || 
      (p.show_non_leaders && modes[i] != MODE_LEADER)) {
      // Set color and width of orientation lines
      glColor4f(0.6f, 0.6f, 0.6f, 0.8f);
      glLineWidth(2.0f);
      glBegin(GL_LINES);
      glVertex3f(positions[i].x, positions[i].y, 0.0f);
      glVertex3f(positions[i].x + ((100.0f * velocities[i].x) / p.vel_bound), 
          positions[i].y + ((100.0f * velocities[i].y) / p.vel_bound), 0.1f);
      glEnd();
    }

    // Set color and width for communication connections
    glColor4f(0.6f, 0.6f, 0.0f, 0.4f);
    glLineWidth(1.0f);

    // Communication connections
    if (p.show_connections) {
      for (uint j = i + 1; j < p.num_robots; j++) {
        if (laplacian[(i * p.num_robots) + j].x == LAPLACIAN_CONNECTED) {
          if (((p.show_leaders && modes[i] == MODE_LEADER) 
                || (p.show_non_leaders && modes[i] != MODE_LEADER)) 
              && ((p.show_leaders && modes[j] == MODE_LEADER) 
                || (p.show_non_leaders && modes[j] != MODE_LEADER))) {
            glBegin(GL_LINES);
            glVertex3f(positions[i].x, positions[i].y, 0.0f);
            glVertex3f(positions[j].x, positions[j].y, 0.0f);
            glEnd();
          }
        }
      }
    }

    // Show communication range if specified in parameters
    if (p.show_range) {
      drawEllipse(positions[i].x, positions[i].y, p.range, p.range, false);
    }
  }
  
  // Refresh display
  glutSwapBuffers();
  glutPostRedisplay();

  // Increment frames shown
  frames++;
}


void screenToWorld(float3 screen, float3 *world) {

  // Initialize variables
  GLint viewport[4];
  GLdouble modelview[16];
  GLdouble projection[16];
  // Screen and world coordinates
  GLdouble x_s, y_s, z_s;
  GLdouble x_w, y_w, z_w;

  // Get view matrix data and viewport bounds
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  glGetIntegerv(GL_VIEWPORT, viewport);

  // Get screen coordinates
  x_s = (double)(screen.x);
  y_s = viewport[3] - (double)(screen.y) - 10.0;
  z_s = (double)(screen.z);

  // Get world coordinates from screen coordinates
  gluUnProject(x_s, y_s, z_s, modelview, projection, viewport, &x_w, &y_w, &z_w);
  world->x = (float)(x_w);
  world->y = (float)(y_w);
  world->z = (float)(z_w);
}

void worldToScreen(float3 world, float3 *screen)
{
  // Transform matricies
  GLint viewport[4];
  GLdouble modelview[16];
  GLdouble projection[16];
  // Screen and world coordinates
  GLdouble x_s, y_s, z_s;
  GLdouble x_w, y_w, z_w;

  // Get view matrix data and viewport bounds
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  glGetIntegerv(GL_VIEWPORT, viewport);

  // Get world coordinates
  x_w = (double)(world.x);
  y_w = (double)(world.y);
  z_w = (double)(world.z);

  gluProject(x_w, y_w, z_w, modelview, projection, viewport, &x_s, &y_s, &z_s);
  screen->x = (float)(x_s);
  screen->y = (float)(y_s);
  screen->z = 0.0f; //(float)(z_s);
}

void resetCamera()
{
  // Reset the camera to the start position
  translate_x0 = -20.0f;
  translate_y0 = 0.0f;
  translate_z0 = 100.0f;
  rotate_x0 = 0.0f;
  rotate_y0 = 0.0f;
}

/****************************
***** HELPER FUNCTIONS ******
****************************/

void calculateFPS(int value)
{
  // Convert frames per second into string for window header
  char fps_text[256];
  //sprintf_s(fps_text, "CUDA Swarm Simulation (%d FPS)", frames);
  sprintf(fps_text, "CUDA Swarm Simulation (%d FPS)", frames);
  glutSetWindowTitle(fps_text);

  // Reset the frames counter
  last_frames = frames;
  frames = 0;

  // Reset timer every second for this function to recalculate FPS
  glutTimerFunc(1000, calculateFPS, 0);
}

void loadParametersFromFile(std::string filename)
{
  std::fstream file(filename);
  std::string str;
  unsigned int line = 1;
  size_t comment;

  // Get the parameters from specified file
  while (std::getline(file, str)) {
    // Remove any comment from the line
    if (comment = str.find('#', 0)) {
      str = str.substr(0, comment);
    }

    // String working variables
    std::istringstream iss(str);
    std::vector<std::string> tokens;

    // Place the parameter and value into the token array
    copy(std::istream_iterator<std::string>(iss), 
        std::istream_iterator<std::string>(), back_inserter(tokens));

    // Ensure valid line before processing parameter
    if (tokens.size() == 2) {
      processParam(tokens);
    }

    // Move to the next line in the parameters file
    line++;
  }
}

void processParam(std::vector<std::string> tokens)
{
  // Assign parameters according to the parameter name
  if (tokens[0] == "align_weight")
    p.align_weight = std::stof(tokens[1]);
  else if (tokens[0] == "ang_bound")
    p.ang_bound = std::stof(tokens[1]);
  else if (tokens[0] == "behavior")
    p.behavior = std::stoul(tokens[1]);
  else if (tokens[0] == "cohere_weight")
    p.cohere_weight = std::stof(tokens[1]);
  else if (tokens[0] == "confirm_quit")
    p.confirm_quit = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "current")
    p.current = std::stof(tokens[1]);
  else if (tokens[0] == "hops")
    p.hops = std::stoul(tokens[1]);
  else if (tokens[0] == "leader_selection")
    p.leader_selection = std::stoul(tokens[1]);
  else if (tokens[0] == "log_data")
    p.log_data = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "range_r")
    p.range_r = std::stof(tokens[1]);
  else if (tokens[0] == "range_f")
    p.range_f = std::stof(tokens[1]);
  else if (tokens[0] == "range_d")
    p.range_d = std::stof(tokens[1]);
  else if (tokens[0] == "range_o")
    p.range_o = std::stof(tokens[1]);
  else if (tokens[0] == "range_l")
    p.range_l = std::stof(tokens[1]);
  else if (tokens[0] == "range")
    p.range = std::stof(tokens[1]);
  else if (tokens[0] == "max_explore")
    p.max_explore = std::stoul(tokens[1]);
  else if (tokens[0] == "max_obstacle_size")
    p.max_obstacle_size = std::stoul(tokens[1]);
  else if (tokens[0] == "noise")
    p.noise = std::stof(tokens[1]);
  else if (tokens[0] == "num_obstacles")
    p.num_obstacles = std::stoul(tokens[1]);
  else if (tokens[0] == "num_robots")
    p.num_robots = std::stoul(tokens[1]);
  else if (tokens[0] == "point_size")
    p.point_size = std::stoul(tokens[1]);
  else if (tokens[0] == "repel_weight")
    p.repel_weight = std::stof(tokens[1]);
  else if (tokens[0] == "add_failures")
    p.add_failures = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "show_ap")
    p.show_ap = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "show_connections")
    p.show_connections = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "show_convex_hull")
    p.show_convex_hull = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "show_explored")
    p.show_explored = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "show_info_graph")
    p.show_info_graph = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "show_leaders")
    p.show_leaders = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "show_range")
    p.show_range = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "highlight_leaders")
    p.highlight_leaders = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "highlight_pioneers")
    p.highlight_pioneers = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "show_non_leaders")
    p.show_non_leaders = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "query_trust")
    p.query_trust = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "start_size")
    p.start_size = std::stof(tokens[1]);
  else if (tokens[0] == "step_limit")
    p.step_limit = std::stoul(tokens[1]);
  else if (tokens[0] == "targets")
    p.targets = std::stoul(tokens[1]);
  else if (tokens[0] == "training")
    p.training = (std::stoul(tokens[1]) != 0);
  else if (tokens[0] == "vel_bound")
    p.vel_bound = std::stof(tokens[1]);
  else if (tokens[0] == "window_height")
    p.window_height = std::stoul(tokens[1]);
  else if (tokens[0] == "window_width")
    p.window_width = std::stoul(tokens[1]);
  else if (tokens[0] == "world_size")
    p.world_size = std::stoul(tokens[1]);
}

void generateWorld()
{
  // Create the specified number of obstacles in the parameters file
  for (uint i = 0; i < p.num_obstacles; i++) {
    bool obstacle_accepted = false;

    // Generate obstacles, discarding ones that don't fit the criteria
    while (!obstacle_accepted) {
      float4 obstacle = make_float4(0.0f, 0.0f, 0.0f, 0.0f);

      // Create width and height for the obstacle
      obstacle.z = (float)(rand() % p.max_obstacle_size);
      obstacle.w = (float)(rand() % p.max_obstacle_size);
      // Create x, y position for top left corner of rectangular obstacle
      obstacle.x = rand() % (p.world_size - (uint)(obstacle.z)) - (p.world_size / 2.0f);
      obstacle.y = rand() % (p.world_size - (uint)(obstacle.w)) - (p.world_size / 2.0f);

      // Ensure obstacle does not cover the start or goal areas and that 
      // it is not too thin
      if ((obstacle.x < -(p.start_size + 1.0f) - obstacle.z || 
        obstacle.x > (p.start_size + 1.0f) ||
        obstacle.y < -(p.start_size + 1.0f) - obstacle.w || 
        obstacle.y > (p.start_size + 1.0f)) &&
        (obstacle.z > 3.0f && obstacle.w > 3.0f)) {
        // Signal the obstacle fits criteria
        obstacle_accepted = true;
        // Add this to the list of obstacles
        obstacles[i] = obstacle;
      }
    }
  }

  // Create the specified number of targets in the parameters file
  for (uint i = 0; i < p.targets; i++) {
    bool target_accepted = false;

    // Generate targets, discarding ones that are within obstacles
    while (!target_accepted) {
      int3 target = make_int3(0, 0, 0);
      float x = rand() % (p.world_size) - (p.world_size / 2.0f);
      float y = rand() % (p.world_size) - (p.world_size / 2.0f);
      target.x = (int)(x);
      target.y = (int)(y);

      // Ensure this target is not within an obstacle
      if (!checkCollision(x, y)) {
        // Signal this obstacle fits the criteria
        target_accepted = true;
        // Add this to the list of targets
        targets[i] = target;
      }
    }
  }
}

void calculateOccupancyGrid()
{
  // Iterate through each cell of the occupancy grid to calculate its value
  float ws_10 = (float)(p.world_size) * 10.0f;
  uint len = (uint)(ws_10 * ws_10);
  for (uint i = 0; i < len; i++) {
    // Get the x and y coordinates for this cell
    float x = ((i % (int)(ws_10)) / 10.0f) - ws_2;
    float y = (floorf((float)(i) / ws_10) / 10.0f) - ws_2;
    occupancy[i] = checkCollision(x, y);
  }
}

bool checkCollision(float x, float y)
{
  for (uint i = 0; i < p.num_obstacles; i++) {
    // Collision occurs if the point to check is within an obstacle or outside 
    // the world boundaries
    if (x >= obstacles[i].x && x < obstacles[i].x + obstacles[i].z 
        && y > obstacles[i].y && y < obstacles[i].y + obstacles[i].w) {
      return true;
    }
  }
  return false;
}

void promptTrust(int a)
{
  // Clear user input that has not yet been sent
  mouse_start_x = 0.0f; mouse_start_y = 0.0f; 
  mouse_last_x = 0.0f; mouse_last_y = 0.0f;
  // Pause and signal trust has not yet been verified
  paused = true;
  trust_verified = false;
}

void updateExplored() {

  // Variables to used in explored grid cell updates
  float world_size_2 = p.world_size / 2.0f;

  // Clear the value for data.explored_this_step
  data.explored_this_step = 0;

  // Update explored grid values
  for (uint i = 0; i < p.world_size * p.world_size; i++) {
    // Skip cells that are fully explored
    if (abs(explored_grid[i]) < (int)(p.max_explore)) {
      // Get the world coordinates for this iteration
      float world_x = -world_size_2 + (float)(floor(i / p.world_size));
      float world_y = -world_size_2 + (float)(i % p.world_size);

      // Only do the following if cell is within range of the swarm bounds
      // and if not paused
      if ((world_x > data.bounds.x - p.range) &&
        (world_x < data.bounds.y + p.range) &&
        (world_y > data.bounds.z - p.range) &&
        (world_y < data.bounds.w + p.range) && !paused) {
        // Check each robot to see if it is within range of this cell
        for (uint n = 0; n < p.num_robots; n++) {
          if (eucl2(world_x + 0.5f, world_y + 0.5f, 
                positions[n].x, positions[n].y) <= p.range) {
            // Increment/decrement based on whether cell is obstacle
            if (explored_grid[i] >= 0) {
              explored_grid[i]++;
            }
            else {
              explored_grid[i]--;
            }

            // Update the area explored this step
            data.explored_this_step++;
          }
        }

        // Restrict the absolute value of explored value to p.max_explore
        if (explored_grid[i] > 0) {
          explored_grid[i] = min(explored_grid[i], (int)(p.max_explore));
        }
        else {
          explored_grid[i] = max(explored_grid[i], (int)(p.max_explore) * -1);
        }
      }
    }
  }
}

void exitSimulation()
{
  // Delete vertex buffer object
  deleteVBO(&vbo_name, cuda_vbo_resource);

  // Free CUDA variables
  cuFree();
  cudaFree(positions);
  cudaFree(velocities);
  cudaFree(modes);
  cudaFree(leaders);
  cudaFree(nearest_leaders);
  cudaFree(leader_countdowns);
  cudaFree(obstacles);
  cudaFree(positions_obs_from_cuda);

  // Free non-CUDA variables
  std::free(explored_grid);
  std::free(ap);
  std::free(targets);
  std::free(occupancy);

  // Close the output and world data files
  if (output_f != NULL) {
    fclose(output_f);
  }

  // Close window and exit
  glutDestroyWindow(0);

  // Wait for keypress to exit (if set in parameters)
  if (p.confirm_quit) {
    std::printf("Enter any key to quit.\n");
    getchar();
  }

  // Reset CUDA device
  cudaDeviceReset();

  std::exit(0);
}

void printDataHeader()
{
  if (p.log_data) {
    // Step data header
    std::fprintf(output_f, "step step_num behavior behavior_data align_weight velocity avg_heading heading_var centroid_x centroid_y convex_hull_area connectivity explored_area targets_seen targets_found trust\n");
    std::fprintf(output_f, "heading direction magnitude\n");
  }
}

/************************
***** PROGRAM LOOP ******
************************/

static void step(int value)
{
  // Only perform a step if not paused
  if (!paused) {
    // Create the output file for logging
    if (!initial_passed) {
      // Print the data column headers
      if (p.log_data) {
        //fopen_s(&output_f, output_fname.str().c_str(), "w");
        output_f = fopen(output_fname.str().c_str(), "w");
        printDataHeader();
      }
      
      // Process data of initial state
      processData(positions, velocities, explored_grid, targets, laplacian, ap, &data, p);

      // Indicates inital state has passed
      initial_passed = true;
    }

    // Determine if simulation should now query user trust
    if (step_num % 1800 == 0 && step_num != 0 && p.query_trust) {
      promptTrust(0);
    }

    // Lower alignment weight to add failure if applicable
    if (p.add_failures) {
      bool is_failure = false;
      for (uint i = 0; i < minutes; i++) {
        if (step_num >= failures[i].x && step_num < failures[i].y) {
          is_failure = true;
          break;
        }
      }
      (is_failure) ? p.align_weight = 0.6f : p.align_weight = 1.0f;
    }
    // Shen
    //printf("%f\n", p.align_weight);

    // Launch the main kernel to perform one simulation step
    //flocking needs goal_vector
    //rendezvous needs goal_point
    //step_num = the time from beginning
    //leaders = whether each member is leader or not, used in kernels.cu
    //ap = Articulation pts (min vertex cut set)
    //p = parameters

    //std::cout<<"goal_vector = ("<<goal_vector.x<<", "<<goal_vector.y<<", "
      //<<goal_vector.z<<")"<<std::endl;
    //std::cout<<"goal_point = ("<<goal_point.x<<", "<<goal_point.y<<")"<<std::endl;

    //goal_vector.x = 10.0f;
    //goal_vector.y = 0.0f;
    //check what is mode and leaders
    //for (int i = 0; i < p.num_robots; i++) {
      //std::cout<<"ID="<<i<<", leaders=" << leaders[i] << ", mode="<<modes[i]<<std::endl;
    //}

    launchMainKernel(goal_vector, goal_point, step_num, leaders, ap, p, 
        &cuda_vbo_resource);

    // Retrieve data from GPU (kernels.cu)
    getData(p.num_robots, positions, velocities, modes, positions_obs_from_cuda);

    /// print positions_obs_from_cuda
    //std::set<int> robots_obs_indices;
    //for (int i = 0; i < p.num_robots*NUM_ANGLE_RAY_TRACE; i ++) {
      //int robot_index = floor(i/NUM_ANGLE_RAY_TRACE);
      ////robots_obs_indices.emplace(robot_index);
      //if (positions_obs_from_cuda[i].w!=-1.0f) {
        //std::cout<<"Robot "<<robot_index<<" encounter obstacle at ("
          //<<positions_obs_from_cuda[i].x<<", "
          //<<positions_obs_from_cuda[i].y<<")"<<std::endl;
      //}
    //}

    //for (std::set<int>::iterator it=robots_obs_indices.begin(); 
        //it!=robots_obs_indices.end(); ++it) {
      //std::cout<<"Robot "<<*it<<" has encountered obstacles."<<std::endl;
    //}
    
    ////x = int \in [-75, 74]
    ////y = int \in [-75, 74]
    //for (uint i = 0; i < p.world_size * p.world_size; i++) {
      //float world_x = -world_size_2 + (float)(floor(i / p.world_size));
      //float world_y = -world_size_2 + (float)(i % p.world_size);
      ////std::cout<<"("<<world_x<<", "<<world_y<<")"<<std::endl;
    //}
    //for (uint i = 0; i < p.num_robots*NUM_ANGLE_RAY_TRACE; i ++) {
      //if (positions_obs_from_cuda[i].w!=-1.0f) {
        //obs_x = positions_obs_from_cuda[i].x;
        //obs_y = positions_obs_from_cuda[i].y;
        ///// We will assign the 4 grid around the exact obstacle position to be obstacles
      //}
    /*}*/

    // Update explored grid
    updateExplored();

    /// Here we can use explored to keep track and make the next plan!!!!!!!!!!!!!!!!!
    //if average robot position is close to obstaacle
    //then change direction

    // Get data variables (data_ops.h)
    processData(positions, velocities, explored_grid, targets, laplacian, ap, &data, p);

    // Update leader list (Very inefficient now, should compute at the same 
    // time as convex hull)
    for (uint i = 0; i < p.num_robots; i++) {
      bool is_in_ch = false;
      for (uint j = 0; j < data.ch.size(); j++) {
        if (positions[i].x == data.ch[j].x && positions[i].y == data.ch[j].y) {
          is_in_ch = true;
          break;
        }
      }
      (is_in_ch) ? leaders[i] = 0 : leaders[i] = 1;
    }

    if (p.log_data) {
      // Write data to the output log at the end of every step
      std::fprintf(output_f, 
          "step %lu %d %f %f %f %f %f %f %f %f %f %d %d %d %d\n", 
          step_num, p.behavior, -goal_heading, p.align_weight, p.vel_bound, 
          data.heading_avg, data.heading_var, data.centroid.x, 
          data.centroid.y, data.ch_area, data.connectivity, data.explored, 
          data.targets_seen, data.targets_explored, data.user_trust);
    }

    // Increment the targets counter
    if (step_num % 60 == 0 && step_num != 0) {
      int ind = (int)((float)step_num / 60.0f);
      heading_var_by_second[ind] = data.heading_var;
      area_by_second[ind] = data.ch_area;
    }
    
    // Increment the simulation step counter
    step_num++;
  }
}

/****************
***** MAIN ******
****************/

int main(int argc, char** argv)
{
  // Reseed RNG
  srand((uint)(time(NULL)));

  ///// PARAMETERS /////
  // If not in playback mode, load the parameters from file, given as first 
  // command line argument;
  // else, load the world file in playback mode
  loadParametersFromFile(argv[1]);

  // Begin paused
  paused = true;
  
  // Half the world size
  ws_2 = (float)(p.world_size) / 2.0f;
  // Open new data file for this trial
  output_fname << argv[2];

  ///// MEMORY ALLOCATION /////
  /// XXX: the memory which does not have to communicate with cuda
  explored_grid = (int*)malloc(p.world_size * p.world_size * sizeof(int));
  ap = (bool*)malloc(p.num_robots * sizeof(bool));
  targets = (int3*)malloc(p.targets * sizeof(int3));
  targets_by_second = (int*)malloc((int)((float)p.step_limit / 60.0f) * sizeof(int));
  memset(targets_by_second, 0, (int)((float)p.step_limit / 60.0f) * sizeof(int));
  heading_var_by_second = (float*)malloc((int)((float)p.step_limit / 60.0f) * sizeof(float));
  memset(heading_var_by_second, 0, (int)((float)p.step_limit / 60.0f) * sizeof(float));
  area_by_second = (float*)malloc((int)((float)p.step_limit / 60.0f) * sizeof(float));
  memset(area_by_second, 0, (int)((float)p.step_limit / 60.0f) * sizeof(float));
  occupancy = (bool*)malloc(p.world_size*10*p.world_size*10*sizeof(bool));
  failures = (uint2*)malloc(5 * sizeof(uint2));

  positions_obs = (float4*)malloc(p.world_size*p.world_size*4*sizeof(float));
  for (uint i = 0; i < p.world_size*p.world_size; i++) {
    positions_obs[i] = make_float4(0.0f, 0.0f, (float)GRID_UNEXPLORED,0.0f);
  }

   
  /// The coordinates of all the obstacles detected by robots
  /// Initialized to be 0. 
  /// Whenever the obstacle is detected, assign a color to it
  //int* obstacles_detected = new int*[p.world_size];

  // Initialize pinned host memory for data arrays
  /// XXX: the memory which has to communicate with cuda
  cudaHostAlloc(&positions, p.num_robots * sizeof(float4), 0);
  cudaHostAlloc(&velocities, p.num_robots * sizeof(float3), 0);
  cudaHostAlloc(&modes, p.num_robots * sizeof(float4), 0);
  cudaHostAlloc(&leaders, p.num_robots * sizeof(int), 0);
  cudaHostAlloc(&nearest_leaders, p.num_robots * sizeof(int), 0);
  cudaHostAlloc(&leader_countdowns, p.num_robots * sizeof(uint), 0);
  cudaHostAlloc(&laplacian, p.num_robots * p.num_robots * sizeof(int4), 0);
  cudaHostAlloc(&obstacles, p.num_obstacles * sizeof(float4), 0);
  /// the positions of all the obstacles
	cudaHostAlloc(&positions_obs_from_cuda, 
      p.num_robots*NUM_ANGLE_RAY_TRACE*sizeof(float4), 0);

  // Fill the leader list with -1 initially
  fill(leaders, leaders + p.num_robots, LEADER_NON_EXIST);

  // Generate failure points within the middle 20s of each minute of simulation 
  minutes = (uint)((float)p.step_limit / 3600.0f);
  for (uint i = 0; i < minutes; i++) {
    uint start = 3600 * i + 1800 + (rand() % 1200 - 600);
    uint2 failure = make_uint2(start, start + 600);
    failures[i] = failure;
  }

  // GPU memory allocation
  cudaAllocate(p);

  ///// OPEN GL INITIALIZATION /////
  // Initialize OpenGL
  initGL(argc, argv);
  // Create vertex buffer object (VBO)
  uint size = p.num_robots*4*sizeof(float);
  //(32+150*150)*4*4
  //printf("size = %d\n",size);
  /// cudaGraphicsMapFlagsWriteDiscard = 
  //    CUDA will only write to and will not read from this resource
  createVBO(&vbo_name, &cuda_vbo_resource, 
      cudaGraphicsMapFlagsWriteDiscard, size);

  // Set camera to default settings
  resetCamera();

  ///// CUDA INITIALIZATION /////
  launchInitKernel(p, &cuda_vbo_resource);
  // Retrieve initial data from GPU (kernels.cu)
  getData(p.num_robots, positions, velocities, modes, positions_obs_from_cuda);
  getLaplacian(p.num_robots, laplacian);

  ///// WORLD GENERATION /////
  generateWorld();

  ///// OCCUPANCY AND EXPLORATION GRID INITIALIZATION /////
  calculateOccupancyGrid();
  // Send occupancy grid to GPU
  setOccupancy(p, occupancy);
  // Create the explored grid
  for (uint i = 0; i < p.world_size * p.world_size; i++) {
    // Get the coordinates of the grid cell
    float y = (float)(i % p.world_size) - ws_2;
    float x = floorf((float)(i) / (float)(p.world_size)) - ws_2;

    // Initialize cell to -1 if it is covered by an obstacle; 0 otherwise
    (checkCollision(x, y)) ? explored_grid[i] = -1 : explored_grid[i] = 0;
  }

  // Initialize goal heading, goal region, and goal point to 0
  goal_heading = 0.0f;
  goal_point = make_float2(0.0f, 0.0f);
  
  ///// START MAIN LOOP /////
  glutMainLoop();

  ///// QUIT /////
  exitSimulation();

  return 0;
}
