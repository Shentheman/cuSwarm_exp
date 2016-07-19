#include "run.h"

/*************************************
***** OpenGL Callback Functions ******
*************************************/

#ifdef GUI
void drawInterface(float window_width, float window_height)
{
	// Only draw edges (wire frame)
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glLineWidth(2.0);

	// Get the current behavior
	int behavior = static_cast<int>(p.behavior);

	// Draw world boundaries
	float world_size_2 = p.world_size / 2.0f;
	glBegin(GL_POLYGON);
	glVertex3f(-world_size_2, -world_size_2, 0.0f);
	glVertex3f(-world_size_2, world_size_2, 0.0f);
	glVertex3f(world_size_2, world_size_2, 0.0f);
	glVertex3f(world_size_2, -world_size_2, 0.0f);
	glEnd();

	// Draw obstacles
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	for (uint i = 0; i < static_cast<uint>(p.num_obstacles); i++) {
		glBegin(GL_POLYGON);
		glVertex3f(obstacles[i].x, obstacles[i].y, 0.0f);
		glVertex3f(obstacles[i].x + obstacles[i].z, obstacles[i].y, 0.0f);
		glVertex3f(obstacles[i].x + obstacles[i].z, 
			obstacles[i].y + obstacles[i].w, 0.0f);
		glVertex3f(obstacles[i].x, obstacles[i].y + obstacles[i].w, 0.0f);
		glEnd();
	}

	// Update explored grid values and draw grid square on screen
	uint world_size = static_cast<uint>(p.world_size);
	uint explore_cell_size = static_cast<uint>(p.explore_cell_size);
	for (uint i = 0; i < world_size; i += explore_cell_size) {
		for (uint j = 0; j < world_size; j += explore_cell_size) {
			// Get the world coordinates for this iteration
			float world_x = static_cast<float>(-world_size_2 + i);
			float world_y = static_cast<float>(-world_size_2 + j);

			// Only do the following if the cell is within range of the swarm bounds
			// Only do the following every 5 steps, and if not paused
			if ((world_x > data[6] - p.max_d) && (world_x < data[7] + p.max_d) &&
				(world_y > data[8] - p.max_d) && (world_y < data[9] + p.max_d) && 
				step_num % 5 == 0 && !paused) {
				// Increment explored value by 1 for each robot within range of this 
				// cell (meaning this cell is seen by the robot)
				for (uint n = 0; n < num_robots; n++) {
					if (eucl2(world_x + 0.5f, world_y + 0.5f, 
						positions[n].x, positions[n].y) <= p.max_d) {
						explored_grid[i][j]++;
					}
				}

				// Restrict the explored value from [0, p.max_explore]
				explored_grid[i][j] = min(explored_grid[i][j],
					static_cast<int>(p.max_explore));
				explored_grid[i][j] = max(explored_grid[i][j], 0);
			}

			// Now draw the grid cell if explored
			float explored_color = 0.4f * (static_cast<float>(explored_grid[i][j]) / 
				p.max_explore);
			if (explored_color > 0.0f) {
				glColor4f(explored_color, explored_color, explored_color, 1.0f);
				glBegin(GL_POLYGON);
				glVertex3f(world_x, world_y, -0.2f);
				glVertex3f(world_x + p.explore_cell_size, world_y, -0.2f);
				glVertex3f(world_x + p.explore_cell_size,
					world_y + p.explore_cell_size, -0.2f);
				glVertex3f(world_x, world_y + p.explore_cell_size, -0.2f);
				glEnd();
			}
		}
	}

	// Set color to cyan for user inputs
	glColor4f(0.0f, 1.0f, 1.0f, 1.0f);

	// Reinitialize OpenGL modelview and projection matricies
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Convert user-drawn line from screen to world coordinates
	float3 screen_start_point = make_float3(mouse_start_x, mouse_start_y, 0.0f);
	float3 screen_last_point = make_float3(mouse_last_x, mouse_last_y, 0.0f);
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

void drawEllipse(float cx, float cy, float w, float h)
{
	// Variables for incremental lines
	float theta = 2.0f * static_cast<float>(PI) / 100.0f;
	float c = cosf(theta);
	float s = sinf(theta);
	float t;
	float x = 1.0f;
	float y = 0.0f;

	// Draw the ellipse
	glBegin(GL_LINE_LOOP);
	for (uint i = 0; i < 100; i++) {
		glVertex2f((w * x) + cx, (h * y) + cy);

		// Apply the rotation matrix
		t = x;
		x = c * x - s * y;
		y = s * t + c * y;
	}
	glEnd();
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
		// Pause / resume the simulation
		paused = !paused;
		break;
	}
	case '1': {
		// Switch to rendezvous
		p.behavior = 0.0f;
		break;
	}
	case '2': {
		// Switch to flocking
		p.behavior = 1.0f;
		break;
	}
	case '3': {
		// Switch to dispersion
		p.behavior = 2.0f;
		break;
	}
	case 27: { // Escape key
		exitSimulation();
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
	if (state == GLUT_DOWN) {	// If the button is pressed down
		// Get mouse button
		mb = button;

		// Zoom in/out if mouse wheel is used
		if (mb == 3) {
			translate_z0 -= 5.0f;
		}
		else if (mb == 4) {
			translate_z0 += 5.0f;
		}

		// Set the user-drawn line start point if the user pressed the primary 
		// mouse button and the simulation is not paused
		if ((!paused) && mb == 0) {
			mouse_start_x = static_cast<float>(x);
			mouse_start_y = static_cast<float>(y);
			mouse_last_x = mouse_start_x;
			mouse_last_y = mouse_start_y;
		}
	}
	else if (state == GLUT_UP) {	// If the button is released
		// Only primary mouse button releases trigger events
		if (mb == 0) {
			// If the simulation is paused and not in an estimation mode, unpause it; 
			// else log the new user goal heading and log the information; 
			// else process the user estimate command
			if (paused) {
				paused = false;
			}
			else if (!paused) {
				// Get the goal direction in radians
				goal_heading = atan2f(static_cast<float>(y) - mouse_start_y, 
					static_cast<float>(x) - mouse_start_x);
				// Transform this into a 2D unit vector (float3, but z is not used)
				goal_vector = make_float3(cosf(goal_heading), 
					-sinf(goal_heading), 0.0f);
#ifdef LOGGING
				logUserHeadingCommand();
#endif
				// Clear the user-drawn line data points
				mouse_start_x = 0;
				mouse_start_y = 0;
				mouse_last_x = 0;
				mouse_last_y = 0;
			}
		}

		// Reset mouse button
		mb = -1;
	}
}

void motion(int x, int y)
{
	// Draw the user heading line if the primary button is down and the simulation 
	// is not paused
	if (mb == 0 && !paused) 
	{
		mouse_last_x = static_cast<float>(x);
		mouse_last_y = static_cast<float>(y);
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

/****************************
***** OPENGL FUNCTIONS ******
****************************/

void initGL(int argc, char **argv)
{
	glutInit(&argc, argv);

	screen_width = static_cast<int>(p.window_width);
	screen_height = static_cast<int>(p.window_height);
	if (screen_width == 0 || screen_height == 0) {
		screen_width = glutGet(GLUT_SCREEN_WIDTH);
		screen_height = glutGet(GLUT_SCREEN_HEIGHT);
	}

	// Setup window
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(screen_width, screen_height);
	glutCreateWindow("CUDA Swarm Simulation");

	// If in Windows, start with the window maximized
#ifdef _WIN32
	// Get the handle for the current window
	HWND win_handle = FindWindow(0, "CUDA Swarm Simulation");
	if (!win_handle)
	{
		printf("!!! Failed FindWindow\n");
	}
	// Maximize the window for full screen simulation operation
	SetWindowLong(win_handle, GWL_STYLE, (GetWindowLong(win_handle, GWL_STYLE) | 
		WS_MAXIMIZE));
	ShowWindowAsync(win_handle, SW_SHOWMAXIMIZED);
#endif

	// Register OpenGL callbacks
	glutDisplayFunc(display);				// OpenGL display callback (looped)
	glutKeyboardFunc(keyboard);				// OpenGL keyboard callback
	glutSpecialFunc(keyboardSpecial);		// OpenGL keyboard special callback
	glutMouseFunc(mouse);					// OpenGL mouse callback
	glutMotionFunc(motion);					// OpenGL mouse motion callback
	glutPassiveMotionFunc(motion);			// OpenGL passive mouse motion callback
	glutTimerFunc(1000, calculateFPS, 0);	// Recalculate FPS every 1/2 second

	// GLEW initialization
	glewInit();
	if (!glewIsSupported("GL_VERSION_2_0")) {
		fprintf(stderr, "ERROR: Support for necessary OpenGL extensions missing.");
		fflush(stderr);
		exit(0);
	}
}

void createVBO(GLuint* vbo, struct cudaGraphicsResource **vbo_res,
	unsigned int vbo_res_flags)
{
	// Create vertex buffer object
	glGenBuffers(1, vbo);
	glBindBuffer(GL_ARRAY_BUFFER, *vbo);

	// Initialize VBO
	uint size = num_robots * 4 * sizeof(float);
	glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Register VBO with CUDA
	cudaGraphicsGLRegisterBuffer(vbo_res, *vbo, vbo_res_flags);
}

void deleteVBO(GLuint *vbo, struct cudaGraphicsResource *vbo_res)
{
	// unregister this buffer object with CUDA
	cudaGraphicsUnregisterResource(vbo_res);

	// Delete VBO
	glBindBuffer(1, *vbo);
	glDeleteBuffers(1, vbo);
	*vbo = 0;
}

static void display(void)
{
	// Take one simulation step
	step(0);

	// Get the world size
	ws_uint = static_cast<uint>(p.world_size);

	// Get the window dimensions
	int window_width = glutGet(GLUT_WINDOW_WIDTH);
	int window_height = glutGet(GLUT_WINDOW_HEIGHT);

	// Clear the display
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, window_width, window_height);

	// Set the environment background
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	// Draw interface elements
	drawInterface(static_cast<float>(window_width), 
		static_cast<float>(window_height));

	// Projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(75.0, (GLfloat)window_width / (GLfloat)window_height, 
		0.001, 500.0);

	// Change point size based on distance from camera, if not in 2D
	glPointSize(p.point_size);
	glEnable(GL_POINT_SMOOTH);
	float quadratic[] = { 0.05f, 0.0f, 0.001f };
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
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDepthMask(GL_TRUE);

	// Transparency
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Draw agents from vbo
	glBindBuffer(GL_ARRAY_BUFFER, vbo_swarm);
	glVertexPointer(3, GL_FLOAT, 16, 0);
	glColorPointer(4, GL_UNSIGNED_BYTE, 16, (GLvoid*)12);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glDrawArrays(GL_POINTS, 0, num_robots);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	// Draw orientation lines
	glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
	glLineWidth(2.0);
	for (uint i = 0; i < num_robots; i++) {
		// Only draw the orientation line if in full information mode, or 
		// leader-only mode with the robot being a leader
		if (p.information_mode == 2.0f ||
			(p.information_mode == 1.0f && modes[i] == 0)) {
			glBegin(GL_LINES);
			glVertex3f(positions[i].x, positions[i].y, 0.0f);
			glVertex3f(positions[i].x + ((100.0f * velocities[i].x) / p.vel_bound),
				positions[i].y + ((100.0f * velocities[i].y) / p.vel_bound), 0.0f);
			glEnd();
		}
	}

	// Refresh display
	glutSwapBuffers();
	glutPostRedisplay();

	// Increment frames shown
	frames++;
}

/****************************
***** HELPER FUNCTIONS ******
****************************/

void calculateFPS(int value)
{
	// Convert frames per second into string for window header
	char fps_text[256];
	sprintf_s(fps_text, "CUDA Swarm Simulation (%d FPS)", frames);
	glutSetWindowTitle(fps_text);

	// Reset the frames counter
	last_frames = frames;
	frames = 0;

	// Reset timer every 1/2 second for this function to recalculate FPS
	glutTimerFunc(1000, calculateFPS, 0);
}

void screenToWorld(float3 screen, float3 *world)
{
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
	x_s = static_cast<double>(screen.x);
	y_s = viewport[3] - static_cast<double>(screen.y) - 10.0;
	z_s = static_cast<double>(screen.z);

	// Get world coordinates from screen coordinates
	gluUnProject(x_s, y_s, z_s, modelview, projection, viewport, &x_w, &y_w, &z_w);
	world->x = static_cast<float>(x_w);
	world->y = static_cast<float>(y_w);
	world->z = 0.0f; //static_cast<float>(z_w);
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
	x_w = static_cast<double>(world.x);
	y_w = static_cast<double>(world.y);
	z_w = static_cast<double>(world.z);

	gluProject(x_w, y_w, z_w, modelview, projection, viewport, &x_s, &y_s, &z_s);
	screen->x = static_cast<float>(x_s);
	screen->y = static_cast<float>(y_s);
	screen->z = 0.0f; //static_cast<float>(z_s);
}

void drawText(float x, float y, float x_scale, float y_scale, const char *string, 
	GLfloat r, GLfloat g, GLfloat b)
{
	// Change to specified color
	glColor4f(r, g, b, 0.75f);

	// Draw text at the given point
	glTranslatef(x, y, 0.0f);
	glScalef(x_scale, y_scale, 1);
	glutStrokeString(GLUT_STROKE_ROMAN, (unsigned char*)string);
}

void resetCamera()
{
	// Reset the camera to the start position
	translate_x0 = 0.0f;
	translate_y0 = 0.0f;
	translate_z0 = 100.0f;
	rotate_x0 = 0.0f;
	rotate_y0 = 0.0f;
}
#endif

float eucl2(float x1, float y1, float x2, float y2)
{
	return sqrtf(powf(x2 - x1, 2.0f) + powf(y2 - y1, 2.0f));
}

void loadParameters(std::string filename)
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

		// Ensure valid line
		if (tokens.size() == 2) {
			// Assign parameters according to the parameter name
			if (tokens[0] == "align_weight")
				p.align_weight = std::stof(tokens[1]);
			else if (tokens[0] == "ang_bound")
				p.ang_bound = std::stof(tokens[1]);
			else if (tokens[0] == "behavior")
				p.behavior = std::stof(tokens[1]);
			else if (tokens[0] == "cohere_weight")
				p.cohere_weight = std::stof(tokens[1]);
			else if (tokens[0] == "current")
				p.current = std::stof(tokens[1]);
			else if (tokens[0] == "explore_cell_size")
				p.explore_cell_size = std::stof(tokens[1]);
			else if (tokens[0] == "hops")
				p.hops = std::stof(tokens[1]);
			else if (tokens[0] == "information_mode")
				p.information_mode = std::stof(tokens[1]);
			else if (tokens[0] == "max_a")
				p.max_a = std::stof(tokens[1]);
			else if (tokens[0] == "max_b")
				p.max_b = std::stof(tokens[1]);
			else if (tokens[0] == "max_c")
				p.max_c = std::stof(tokens[1]);
			else if (tokens[0] == "max_d")
				p.max_d = std::stof(tokens[1]);
			else if (tokens[0] == "max_explore")
				p.max_explore = std::stof(tokens[1]);
			else if (tokens[0] == "max_obstacle_size")
				p.max_obstacle_size = std::stof(tokens[1]);
			else if (tokens[0] == "noise")
				p.noise = std::stof(tokens[1]);
			else if (tokens[0] == "num_obstacles")
				p.num_obstacles = std::stof(tokens[1]);
			else if (tokens[0] == "num_robots")
				p.num_robots = std::stof(tokens[1]);
			else if (tokens[0] == "point_size")
				p.point_size = std::stof(tokens[1]);
			else if (tokens[0] == "repel_weight")
				p.repel_weight = std::stof(tokens[1]);
			else if (tokens[0] == "update_period")
				p.update_period = std::stof(tokens[1]);
			else if (tokens[0] == "vel_bound")
				p.vel_bound = std::stof(tokens[1]);
			else if (tokens[0] == "window_height")
				p.window_height = std::stof(tokens[1]);
			else if (tokens[0] == "window_width")
				p.window_width = std::stof(tokens[1]);
			else if (tokens[0] == "world_size")
				p.world_size = std::stof(tokens[1]);
		}
		else {
			printf("Error reading parameters: line %d has too many tokens (%d)\n",
				line, tokens.size());
		}

		// Move to the next line in the parameters file
		line++;
	}
}

void generateObstacles()
{
	// Get the world size for future use
	int world_size = static_cast<int>(p.world_size);
	int max_obstacle_size = static_cast<int>(p.max_obstacle_size);
	float start_size = sqrtf(static_cast<float>(num_robots)) * 1.5f / 1.5f;

	// Create the specified number of obstacles in the parameters file
	for (uint i = 0; i < static_cast<uint>(p.num_obstacles); i++) {
		bool obstacle_accepted = false;

		// Generate obstacles, discarding ones that don't fit the criteria
		while (!obstacle_accepted) {
			float4 obstacle = make_float4(0.0f, 0.0f, 0.0f, 0.0f);

			// Create width and height for the obstacle
			obstacle.z = static_cast<float>(rand() % max_obstacle_size);
			obstacle.w = static_cast<float>(rand() % max_obstacle_size);
			// Create x, y position for top left corner of rectangular obstacle
			obstacle.x = rand() % (world_size - static_cast<uint>(obstacle.z)) -
				(world_size / 2.0f);
			obstacle.y = rand() % (world_size - static_cast<uint>(obstacle.w)) -
				(world_size / 2.0f);

			// Ensure this obstacle does not cover the start or goal areas and that 
			// it is not too thin
			if ((obstacle.x < -start_size - obstacle.z || obstacle.x > start_size ||
				obstacle.y < -start_size - obstacle.w || obstacle.y > start_size) &&
				(obstacle.z > 3.0f && obstacle.w > 3.0f)) {
				// Signal the obstacle fits criteria
				obstacle_accepted = true;
				// Add this to the list of obstacles
				obstacles[i] = obstacle;
			}
		}
	}
}

void calculateOccupancyGrid()
{
	// Iterate through each cell of the occupancy grid to calculate its value
	float ws_10 = p.world_size * 10.0f;
	uint len = static_cast<uint>(ws_10 * ws_10);
	for (uint i = 0; i < len; i++) {
		// Get the x and y coordinates for this cell
		float x = ((i % static_cast<int>(ws_10)) / 10.0f) - ws_2;
		float y = (floorf(static_cast<float>(i) / ws_10) / 10.0f) - ws_2;
		occupancy[i] = checkCollision(x, y);
	}
}

bool checkCollision(float x, float y)
{
	for (uint i = 0; i < static_cast<uint>(p.num_obstacles); i++) {
		// Collision occurs if the point to check is within an obstacle or outside 
		// the world boundaries
		if (x > obstacles[i].x && x < obstacles[i].x + obstacles[i].z &&
			y > obstacles[i].y && y < obstacles[i].y + obstacles[i].w) {
			return true;
		}
	}
	return false;
}

void exitSimulation()
{
#ifdef GUI
	// Free OpenGL buffer object
	deleteVBO(&vbo_swarm, cuda_vbo_resource);
#endif
	// Free CUDA and data variables
	cuFree();
	cudaFree(positions);
	cudaFree(velocities);
	cudaFree(modes);
	cudaFree(leaders);
	cudaFree(nearest_leaders);
	cudaFree(leader_countdowns);
	cudaFree(obstacles);
	cudaFree(occupancy);
	cudaFree(data);

#ifdef LOGGING
	// Close the data output file
	fclose(output);
#endif

	// Reset CUDA device
	cudaDeviceReset();

	// Close window and exit
#ifdef GUI
	glutDestroyWindow(0);
#endif
	std::exit(0);
}

#ifdef LOGGING
void printDataHeader()
{
	// Step data header
	fprintf(output, "step step_num dist_to_goal avg_heading heading_var ");
	fprintf(output, "centroid_x centroid_y convex_hull_area\n");
	// Heading command data header
	fprintf(output, "heading_command step_num goal_heading\n");
}
#endif

#if defined(GUI) && defined(LOGGING)
void logUserHeadingCommand()
{
	// Note: goal heading is negative due to flipped OpenGL coordinate space
	fprintf(output, "heading_command %d %6.4f\n", step_num, -goal_heading);
}
#endif

/************************
***** PROGRAM LOOP ******
************************/

static void step(int value)
{
	// Only perform a step if not paused
	if (!paused) {
		// Create the output file for logging
#ifdef LOGGING
		if (!log_created) {
			fopen_s(&output, filename.str().c_str(), "w");

			// Print the data column headers
			printDataHeader();
			log_created = true;
		}
#endif
		// Launch the main kernel to perform one simulation step
#ifdef GUI
		launchMainKernel(goal_vector, step_num, p, &cuda_vbo_resource);
#else
		launchMainKernel(goal_vector, step_num, p);
#endif

		// Retrieve data from GPU (kernels.cu)
		getData(num_robots, positions, velocities, modes);
		// Get data variables (data_ops.h)
		processData(num_robots, positions, velocities, data);
		
		// Create point array for convex hull calculations
		vector<Point> points;
		for (uint i = 0; i < num_robots; i++) {
			Point p;
			p.x = positions[i].x;
			p.y = positions[i].y;
			points.push_back(p);
		}
		// Get convex hull of the robot positions in screen coordinates
		robot_ch = convexHull(points);
		// Get the area of the convex hull
		float ch_area = convexHullArea(robot_ch);

#ifdef LOGGING
		// Write data to the output log at the end of every step
		fprintf(output, "step %d %6.4f %6.4f %6.4f %6.4f %6.4f\n", 
			step_num, data[0], data[1], data[2], data[3], ch_area);
#endif

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
	srand(static_cast<uint>(time(NULL)));

	// Load the parameters from file, given as first command line argument
	loadParameters(argv[1]);

	// Get the number of robots for the simulation
	num_robots = static_cast<uint>(p.num_robots);
	// Get the world size for this simulation
	ws_uint = static_cast<uint>(p.world_size);
	ws_2 = p.world_size / 2.0f;

	// Create the explored grid
	explored_grid = new int*[ws_uint];
	for (uint i = 0; i < ws_uint; i++) {
		explored_grid[i] = new int[ws_uint];
	}

	// Begin paused if showing the GUI, otherwise start immediately
#ifdef GUI
	paused = true;
#else
	paused = false;
#endif

	// Open new data file for this trial
#ifdef LOGGING
	filename << argv[2];
#endif

	// Allocate and initialize data vector
	data = (float*) malloc(16 * sizeof(float));
	occupancy = (bool*) malloc(ws_uint * 10 * ws_uint * 10 * sizeof(bool));

	// Initialize pinned host memory for data arrays
	cudaHostAlloc(&positions, num_robots * sizeof(float4), 0);
	cudaHostAlloc(&velocities, num_robots * sizeof(float3), 0);
	cudaHostAlloc(&modes, num_robots * sizeof(float4), 0);
	cudaHostAlloc(&leaders, 16 * sizeof(int), 0);
	cudaHostAlloc(&nearest_leaders, num_robots * sizeof(int), 0);
	cudaHostAlloc(&leader_countdowns, num_robots * sizeof(uint), 0);
	cudaHostAlloc(&obstacles, static_cast<uint>(p.num_obstacles) *
		sizeof(float4), 0);

	// Initialize leader list
	std::fill(leaders, leaders + 10, -1);
	
	// Generate random obstacle positions
	generateObstacles();
	// Compute occupancy grid from obstacles
	calculateOccupancyGrid();

	// Send parameters to GPU
	cudaAllocate(p, occupancy);

#ifdef GUI
	// Initialize OpenGL
	initGL(argc, argv);
	// Create vertex buffer object (VBO)
	createVBO(&vbo_swarm, &cuda_vbo_resource, cudaGraphicsMapFlagsWriteDiscard);
	// Set camera to default settings
	resetCamera();
#endif

	// Launch initialization kernel
#ifdef GUI
	launchInitKernel(p, &cuda_vbo_resource);
#else
	launchInitKernel(p);
#endif
	// Retrieve data from GPU (kernels.cu)
	getData(num_robots, positions, velocities, modes);
	
	// Start the main loop
#ifdef GUI
	glutMainLoop();
#else
	////////////////////////////////////////////////
	///// REPLACE THIS WITH PLANNING ALGORITHM /////
	////////////////////////////////////////////////
	while (true) {
		step(0);
	}
#endif

	return 0;
}
