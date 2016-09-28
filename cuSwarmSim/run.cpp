#include "run.h"

/*************************************
***** OpenGL Callback Functions ******
*************************************/

void drawInterface(float window_width, float window_height)
{
	// Only draw edges (wire frame)
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glLineWidth(2.0f);

	// Draw world boundaries
	float world_size_2 = p.world_size / 2.0f;
	glBegin(GL_POLYGON);
	glVertex3f(-world_size_2, -world_size_2, 0.0f);
	glVertex3f(-world_size_2, world_size_2, 0.0f);
	glVertex3f(world_size_2, world_size_2, 0.0f);
	glVertex3f(world_size_2, -world_size_2, 0.0f);
	glEnd();

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Draw explored grid cells on the GUI
	for (uint i = 0; i < p.world_size * p.world_size; i++) {
		// Get the world coordinates for this iteration
		float world_x = -world_size_2 + 
			static_cast<float>(floor(i / p.world_size));
		float world_y = -world_size_2 + static_cast<float>(i % p.world_size);

		// Now draw the grid cell if explored
		float explored_color = 0.0f;
		if (explored_grid[i] != 0) {
			explored_color = fabsf(static_cast<float>(explored_grid[i]) /
				p.max_explore);
			// Color is based on obstacle/free space
			if (explored_grid[i] > 0) {		// Free space
				glColor4f(0.1f * explored_color, 0.3f * explored_color, 
					0.6f * explored_color, 1.0f);
			}
			else {							// Obstacle
				// Lower bar for showing an obstacle cell as fully explored
				explored_color = min(1.0f, explored_color * 4.0f);
				glColor4f(0.6f * explored_color, 0.2f * explored_color,
					0.2f * explored_color, 1.0f);
			}
			glBegin(GL_POLYGON);
			glVertex3f(world_x, world_y, -0.2f);
			glVertex3f(world_x + 1.0f, world_y, -0.2f);
			glVertex3f(world_x + 1.0f, world_y + 1.0f, -0.2f);
			glVertex3f(world_x, world_y + 1.0f, -0.2f);
			glEnd();
		}
	}

	// Draw targets on the GUI
	for (uint i = 0; i < p.targets; i++) {
		// Get the explored grid index that this target corresponds to
		uint exp_ind = static_cast<uint>(((targets[i].x + ws_2) * p.world_size) + 
			(targets[i].y + ws_2));
		// Set the target color based on explored value
		float target_color = fabsf(static_cast<float>(explored_grid[exp_ind]) /
			p.max_explore);

		// Change target color based on whether fully explored
		if (target_color < 1.0f) {
			// Purple (not fully explored)
			glColor4f(0.6f * target_color, 0.0f * target_color,
				0.6f * target_color, 1.0f);
		}
		else {
			// Green (fully explored)
			glColor4f(0.2f * target_color, 0.8f * target_color,
				0.2f * target_color, 1.0f);
			// If first time reaching fully explored status for target, indicate 
			// so in array and add bonus to score
			if (targets[i].z == 0) {
				targets[i].z = 1;
				data.targets_explored++;
			}
		}

		// Draw target
		float x = static_cast<float>(targets[i].x);
		float y = static_cast<float>(targets[i].y);
		glBegin(GL_POLYGON);
		glVertex3f(x, y, -0.1f);
		glVertex3f(x + 1.0f, y, -0.1f);
		glVertex3f(x + 1.0f, y + 1.0f, -0.1f);
		glVertex3f(x, y + 1.0f, -0.1f);
		glEnd();
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
		// Switch to rendezvous if not in automated mode
		if (p.op_mode == 2) {
			p.behavior = 0;
			if (p.log_data) {
				fprintf(world_f, "d %d %4.2f %4.2f %d %4.2f\n", p.behavior, 0.0f, 
					p.vel_bound, step_num, data.score);
			}
		}
		break;
	}
	case '2': {
		// Switch to flocking if not in automated mode
		if (p.op_mode == 2) {
			p.behavior = 1;
			if (p.log_data) {
				fprintf(world_f, "d %d %4.2f %4.2f %d %4.2f\n", p.behavior, 
					goal_heading, p.vel_bound, step_num, data.score);
			}
		}
		break;
	}
	case '3': {
		// Switch to dispersion if not in automated mode
		if (p.op_mode == 2) {
			p.behavior = 2;
			if (p.log_data) {
				fprintf(world_f, "d %d %4.2f %4.2f %d %4.2f\n", p.behavior, 0.0f,
					p.vel_bound, step_num, data.score);
			}
		}
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

		if (mb == 0) { // Left click
			// Set the user-drawn line start point if the user pressed the primary 
			// mouse button and the simulation is not paused
			if ((!paused) && p.op_mode == 2 && mb == 0) {
				mouse_start_x = static_cast<float>(x);
				mouse_start_y = static_cast<float>(y);
				mouse_last_x = mouse_start_x;
				mouse_last_y = mouse_start_y;
			}
		}
		else if (mb == 3) {		// Scroll wheel forward to increase speed
			p.vel_bound = min(4.0f, p.vel_bound + 0.1f);
			fprintf(world_f, "d %d %4.2f %4.2f %d %4.2f\n", p.behavior,
				goal_heading, p.vel_bound, step_num, data.score);
		}
		else if (mb == 4) {		// Scroll wheel backward to decrease speed
			p.vel_bound = max(1.0f, p.vel_bound - 0.1f);
			fprintf(world_f, "d %d %4.2f %4.2f %d %4.2f\n", p.behavior,
				goal_heading, p.vel_bound, step_num, data.score);
		}
		
	}
	else if (state == GLUT_UP) {	// If the button is released
		if (mb == 0) {	// Left click
			// If the simulation is paused, unpause it; 
			// else log the new user goal heading and log the information; 
			if (paused) {
				paused = false;
			}
			else if (!paused && p.op_mode == 2) {
				// Get the goal direction in radians
				goal_heading = atan2f(static_cast<float>(y) - mouse_start_y, 
					static_cast<float>(x) - mouse_start_x);
				// Transform this into a 2D unit vector (float3, but z not used)
				goal_vector = make_float3(cosf(goal_heading), 
					-sinf(goal_heading), 0.0f);

				// Clear the user-drawn line data points
				mouse_start_x = 0;
				mouse_start_y = 0;
				mouse_last_x = 0;
				mouse_last_y = 0;

				// Log goal heading to world file
				if (p.log_data) {
					fprintf(world_f, "d %d %4.2f %4.2f %d %4.2f\n", p.behavior,
						goal_heading, p.vel_bound, step_num, data.score);
				}
			}
		}
		else if (mb == 5) {		// Scroll wheel forward to increase speed
			p.vel_bound = min(4.0f, p.vel_bound + 0.1f);
			fprintf(world_f, "d %d %4.2f %4.2f %d %4.2f\n", p.behavior,
				goal_heading, p.vel_bound, step_num, data.score);
		}
		else if (mb == 6) {		// Scroll wheel backward to decrease speed
			p.vel_bound = max(1.0f, p.vel_bound - 0.1f);
			fprintf(world_f, "d %d %4.2f %4.2f %d %4.2f\n", p.behavior,
				goal_heading, p.vel_bound, step_num, data.score);
		}

		// If mouse_up event cause by scrolling while left mouse is down, reset 
		// mb to 0 (left click); else, reset to null
		if (mb == 5 || mb == 6) {
			mb = 0;
		}
		else {
			mb = -1;
		}
	}
}

void motion(int x, int y)
{
	// Draw the user heading line if the primary button is down and simulation 
	// is not paused
	if (mb == 0 && p.op_mode == 2 && !paused) {
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
	if (!win_handle)
	{
		std::printf("!!! Failed FindWindow\n");
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
	glutPassiveMotionFunc(motion);			// OpenGL pass. mouse motion callback
	glutTimerFunc(1000, calculateFPS, 0);	// Recalculate FPS every 1/2 second

	// GLEW initialization
	glewInit();
	if (!glewIsSupported("GL_VERSION_2_0")) {
		fprintf(stderr, 
			"ERROR: Support for necessary OpenGL extensions missing.");
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
	uint size = p.num_robots * 4 * sizeof(float);
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
	if (p.op_mode != 0) { // Quit if not automated and last step reached
		if (step_num > p.step_limit) {
			exitSimulation();
		}
		else {
			step(0);
		}
	}

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

	// Change point size based on distance from camera
	glPointSize(static_cast<float>(p.point_size));
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
	glDrawArrays(GL_POINTS, 0, p.num_robots);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	// Draw orientation lines
	if (p.information_mode != 0) {
		glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
		glLineWidth(2.0f);
		for (uint i = 0; i < p.num_robots; i++) {
			// Only draw the orientation line if in full information mode, or 
			// leader-only mode with the robot being a leader
			if (p.information_mode == 2 ||
				(p.information_mode == 1 && modes[i] == 0)) {
				glBegin(GL_LINES);
				glVertex3f(positions[i].x, positions[i].y, 0.0f);
				glVertex3f(positions[i].x + ((100.0f * velocities[i].x) / 
					p.vel_bound), positions[i].y + ((100.0f * velocities[i].y) / 
					p.vel_bound), 0.0f);
				glEnd();
			}
		}
	}

	// Draw communication graph
	if (p.information_mode != 0) {
		glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
		glLineWidth(1.0f);
		for (uint i = 0; i < p.num_robots - 1; i++) {
			for (uint j = i + 1; j < p.num_robots; j++) {
				if (laplacian[(i * p.num_robots) + j].w == -1) {
					// Only draw orientation line if in full information mode, 
					// or leader-only mode with the robot being a leader
					if (p.information_mode == 2 ||
						(p.information_mode == 1 && 
						modes[i] == 0 && modes[j] == 0)) {
						glBegin(GL_LINES);
						glVertex3f(positions[i].x, positions[i].y, 0.0f);
						glVertex3f(positions[j].x, positions[j].y, 0.0f);
						glEnd();
					}
				}
			}
		}
	}
	

	// Refresh display
	glutSwapBuffers();
	glutPostRedisplay();

	// Increment frames shown
	frames++;
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
	gluUnProject(x_s, y_s, z_s, modelview, projection, viewport, 
		&x_w, &y_w, &z_w);
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

void resetCamera()
{
	// Reset the camera to the start position
	translate_x0 = 0.0f;
	translate_y0 = 0.0f;
	translate_z0 = 70.0f;
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
	sprintf_s(fps_text, "CUDA Swarm Simulation (%d FPS)", frames);
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
	else if (tokens[0] == "information_mode")
		p.information_mode = std::stoul(tokens[1]);
	else if (tokens[0] == "leader_selection")
		p.leader_selection = std::stoul(tokens[1]);
	else if (tokens[0] == "log_data")
		p.log_data = (std::stoul(tokens[1]) != 0);
	else if (tokens[0] == "max_a")
		p.max_a = std::stof(tokens[1]);
	else if (tokens[0] == "max_b")
		p.max_b = std::stof(tokens[1]);
	else if (tokens[0] == "max_c")
		p.max_c = std::stof(tokens[1]);
	else if (tokens[0] == "max_d")
		p.max_d = std::stof(tokens[1]);
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
	else if (tokens[0] == "op_mode")
		p.op_mode = std::stoul(tokens[1]);
	else if (tokens[0] == "point_size")
		p.point_size = std::stoul(tokens[1]);
	else if (tokens[0] == "repel_weight")
		p.repel_weight = std::stof(tokens[1]);
	else if (tokens[0] == "step_limit")
		p.step_limit = std::stoul(tokens[1]);
	else if (tokens[0] == "targets")
		p.targets = std::stoul(tokens[1]);
	else if (tokens[0] == "update_period")
		p.update_period = std::stoul(tokens[1]);
	else if (tokens[0] == "vel_bound")
		p.vel_bound = std::stof(tokens[1]);
	else if (tokens[0] == "window_height")
		p.window_height = std::stoul(tokens[1]);
	else if (tokens[0] == "window_width")
		p.window_width = std::stoul(tokens[1]);
	else if (tokens[0] == "world_size")
		p.world_size = std::stoul(tokens[1]);
}

void generateWorld(char* filepath)
{
	// Get the starting area
	float start_size = sqrtf(static_cast<float>(p.num_robots)) * 1.5f / 1.5f;

	// Open world file to log obstacle data
	if (p.log_data != 0.0f) {
		fopen_s(&world_f, filepath, "w");
	}

	// Do not write to world file if in playback mode
	if (p.log_data && (p.op_mode != 1)) {
		// Add parameters to world data file
		fprintf(world_f, "align_weight %4.2f\n", p.align_weight);
		fprintf(world_f, "ang_bound %4.2f\n", p.ang_bound);
		fprintf(world_f, "behavior %d\n", p.behavior);
		fprintf(world_f, "cohere_weight %4.2f\n", p.cohere_weight);
		fprintf(world_f, "confirm_quit %d", p.confirm_quit);
		fprintf(world_f, "current %4.2f\n", p.current);
		fprintf(world_f, "hops %d\n", p.hops);
		fprintf(world_f, "information_mode %d\n", p.information_mode);
		fprintf(world_f, "leader_selection %d\n", p.leader_selection);
		fprintf(world_f, "log_data %d\n", p.log_data);
		fprintf(world_f, "max_a %4.2f\n", p.max_a);
		fprintf(world_f, "max_b %4.2f\n", p.max_b);
		fprintf(world_f, "max_c %4.2f\n", p.max_c);
		fprintf(world_f, "max_d %4.2f\n", p.max_d);
		fprintf(world_f, "max_explore %d\n", p.max_explore);
		fprintf(world_f, "max_obstacle_size %d\n", p.max_obstacle_size);
		fprintf(world_f, "noise %4.2f\n", p.noise);
		fprintf(world_f, "num_obstacles %d\n", p.num_obstacles);
		fprintf(world_f, "num_robots %d\n", p.num_robots);
		fprintf(world_f, "op_mode %d\n", 1); // Set op_mode to 1 (for replay)
		fprintf(world_f, "point_size %d\n", p.point_size);
		fprintf(world_f, "repel_weight %4.2f\n", p.repel_weight);
		fprintf(world_f, "step_limit %d\n", p.step_limit);
		fprintf(world_f, "targets %d\n", p.targets);
		fprintf(world_f, "update_period %d\n", p.update_period);
		fprintf(world_f, "vel_bound %4.2f\n", p.vel_bound);
		fprintf(world_f, "window_height %d\n", p.window_height);
		fprintf(world_f, "window_width %d\n", p.window_width);
		fprintf(world_f, "world_size %d\n", p.world_size);

		// Add robot initial positions to world data file
		for (uint i = 0; i < p.num_robots; i++) {
			fprintf(world_f, "r %d %4.2f %4.2f\n", i, positions[i].x,
				positions[i].y);
		}
	}

	// Create the specified number of obstacles in the parameters file
	for (uint i = 0; i < p.num_obstacles; i++) {
		bool obstacle_accepted = false;

		// Generate obstacles, discarding ones that don't fit the criteria
		while (!obstacle_accepted) {
			float4 obstacle = make_float4(0.0f, 0.0f, 0.0f, 0.0f);

			// Create width and height for the obstacle
			obstacle.z = static_cast<float>(rand() % p.max_obstacle_size);
			obstacle.w = static_cast<float>(rand() % p.max_obstacle_size);
			// Create x, y position for top left corner of rectangular obstacle
			obstacle.x = rand() % (p.world_size - static_cast<uint>(obstacle.z)) -
				(p.world_size / 2.0f);
			obstacle.y = rand() % (p.world_size - static_cast<uint>(obstacle.w)) -
				(p.world_size / 2.0f);

			// Ensure obstacle does not cover the start or goal areas and that 
			// it is not too thin
			if ((obstacle.x < -start_size - obstacle.z || 
				obstacle.x > start_size ||
				obstacle.y < -start_size - obstacle.w || 
				obstacle.y > start_size) &&
				(obstacle.z > 3.0f && obstacle.w > 3.0f)) {
				// Signal the obstacle fits criteria
				obstacle_accepted = true;
				// Add this to the list of obstacles
				obstacles[i] = obstacle;
				// Add this obstacle to world data file if not in playback mode
				if (p.log_data && (p.op_mode != 1)) {
					fprintf(world_f, "o %4.2f %4.2f %4.2f %4.2f\n", obstacle.x,
						obstacle.y, obstacle.z, obstacle.w);
				}
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
			target.x = static_cast<int>(x);
			target.y = static_cast<int>(y);

			// Ensure this target is not within an obstacle
			if (!checkCollision(x, y)) {
				// Signal this obstacle fits the criteria
				target_accepted = true;
				// Add this to the list of targets
				targets[i] = target;
				// Add this target to the world data file if not in playback mode
				if (p.log_data && (p.op_mode != 1)) {
					fprintf(world_f, "t %d %d\n", target.x, target.y);
				}
			}
		}
	}
}

void loadSavedMap(char* filepath)
{
	std::fstream file(filepath);
	std::string str;
	size_t comment;

	// Temporary vector for holding obstacles
	vector<float4> o_temp;
	vector<int3> t_temp;

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

		// Load parameter, obstacle, decision, or robot position
		if (tokens.size() == 2) {		// Parameter
			processParam(tokens);
		}
		else if (tokens[0] == "o") {	// Obstacle
			float4 o;
			o.x = stof(tokens[1]);
			o.y = stof(tokens[2]);
			o.z = stof(tokens[3]);
			o.w = stof(tokens[4]);
			o_temp.push_back(o);
		}
		else if (tokens[0] == "t") {	// Target
			int3 t;
			t.x = stoi(tokens[1]);
			t.y = stoi(tokens[2]);
			t.z = 0;
			t_temp.push_back(t);
		}
		else if (tokens[0] == "d") {	// Decision
			Decision d;
			d.behavior = stoul(tokens[1]);
			d.flock_dir = stof(tokens[2]);
			d.velocity = stof(tokens[3]);
			d.time_start = stoul(tokens[4]);
			d.score_start = stof(tokens[5]);
			sequence.push_back(d);
		}
		else if (tokens[0] == "r") {	// Robot
			uint index = stoul(tokens[1]);
			positions[index].x = stof(tokens[2]);
			positions[index].y = stof(tokens[3]);
		}
	}

	// Add obstacles and targets to the simulation
	for (uint i = 0; i < o_temp.size(); i++) {
		obstacles[i] = o_temp[i];
	}
	for (uint i = 0; i < t_temp.size(); i++) {
		targets[i] = t_temp[i];
	}
}

void calculateOccupancyGrid()
{
	// Iterate through each cell of the occupancy grid to calculate its value
	float ws_10 = static_cast<float>(p.world_size) * 10.0f;
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
	for (uint i = 0; i < p.num_obstacles; i++) {
		// Collision occurs if the point to check is within an obstacle or outside 
		// the world boundaries
		if (x >= obstacles[i].x && x < obstacles[i].x + obstacles[i].z &&
			y > obstacles[i].y && y < obstacles[i].y + obstacles[i].w) {
			return true;
		}
	}
	return false;
}

void updateExplored()
{
	// Variables to used in explored grid cell updates
	float world_size_2 = p.world_size / 2.0f;

	// Update explored grid values
	for (uint i = 0; i < p.world_size * p.world_size; i++) {
		// Skip cells that are fully explored
		if (abs(explored_grid[i]) < static_cast<int>(p.max_explore)) {
			// Get the world coordinates for this iteration
			float world_x = -world_size_2 + 
				static_cast<float>(floor(i / p.world_size));
			float world_y = -world_size_2 + static_cast<float>(i % p.world_size);

			// Only do the following if cell is within range of the swarm bounds
			// Only do the following every 5 steps, and if not paused
			if ((world_x > data.bounds.x - p.max_d) &&
				(world_x < data.bounds.y + p.max_d) &&
				(world_y > data.bounds.z - p.max_d) &&
				(world_y < data.bounds.w + p.max_d) &&
				step_num % 5 == 0 && !paused) {
				// Check each robot to see if it is within range of this cell
				for (uint n = 0; n < p.num_robots; n++) {
					if (eucl2(world_x + 0.5f, world_y + 0.5f,
						positions[n].x, positions[n].y) <= p.max_d) {
						// Increment/decrement based on whether cell is obstacle
						if (explored_grid[i] >= 0) {
							explored_grid[i]++;
						}
						else {
							explored_grid[i]--;
						}
					}
				}

				// Restrict the absolute value of explored value to p.max_explore
				if (explored_grid[i] > 0) {
					explored_grid[i] = min(explored_grid[i],
						static_cast<int>(p.max_explore));
				}
				else {
					explored_grid[i] = max(explored_grid[i],
						static_cast<int>(p.max_explore) * -1);
				}
			}
		}
	}
}

void exitSimulation()
{
	if (p.op_mode != 0) {
		// Free OpenGL buffer object
		deleteVBO(&vbo_swarm, cuda_vbo_resource);
	}

	// Free CUDA variables
	cuFree();
	cudaFree(positions);
	cudaFree(velocities);
	cudaFree(modes);
	cudaFree(leaders);
	cudaFree(nearest_leaders);
	cudaFree(leader_countdowns);
	cudaFree(obstacles);

	// Free non-CUDA variables
	std::free(occupancy);
	std::free(explored_grid);

	// Close the output and world data files
	if (output_f != NULL) {
		fclose(output_f);
	}
	if (world_f != NULL) {
		fclose(world_f);
	}

	// Close window and exit
	if (p.op_mode != 0) {
		glutDestroyWindow(0);
	}

	// Show final score if not in automated mode
	if (p.op_mode == 1 || p.op_mode == 2) {
		std::printf("Score: %1.0f\n", data.score);
	}
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
		fprintf(output_f, "step step_num behavior behavior_data velocity ");
		fprintf(output_f, "avg_heading heading_var centroid_x centroid_y ");
		fprintf(output_f, "convex_hull_area connectivity explored_area ");
		fprintf(output_f, "targets_explored score\n");
	}
}

/*****************************************
***** AUTOOMATION LOOP (PLANNING_H) ******
*****************************************/

float getBestHeading()
{
	float2 components = make_float2(0.0f, 0.0f);
	for (uint i = 0; i < p.world_size * p.world_size; i++) {
		// Get the coordinates of the grid cell
		float y = static_cast<float>(i % p.world_size) - ws_2;
		float x = floorf(static_cast<float>(i) / 
			static_cast<float>(p.world_size)) - ws_2;

		// Only analyze cells the centroid could reach within in 10 seconds
		if (eucl2(x, y, data.centroid.x, data.centroid.y) < p.vel_bound * 10.0f) {
			// Add X and Y components to running sum, weighted by explored value
			components.x += static_cast<float>(explored_grid[i]) * 
				(x - data.centroid.x);
			components.y += static_cast<float>(explored_grid[i]) *
				(y - data.centroid.y);
		}
	}

	// Return the resulting angle from the component sums
	float angle = atan2f(components.y, components.x);
	return angle;
}

void printDecisionSequence(vector<Decision> b_seq)
{
	// Print out behavior sequence and score
	std::printf("Score: %4.2f\nBehavior Seq: ", 
		b_seq[b_seq.size() - 1].score_end);
	for (uint j = 0; j < b_seq.size(); j++) {
		std::printf("(%d %4.2f %4.2f (%d-%d) %4.2f) ", b_seq[j].behavior, 
			b_seq[j].flock_dir, b_seq[j].velocity, b_seq[j].time_start, 
			b_seq[j].time_end, b_seq[j].score_end);
	}
	std::printf("\n\n");
}

void automateExplore()
{
	// Begin the clock
	const clock_t comp_time = clock();

	// Best sequence so far
	vector<Decision> best_seq;
	bool goal_reached = false;

	// Queue for holding candidate states
	priority_queue<SwarmState*, vector<SwarmState*>, Compare> queue;
	float high_score = 0.0f;

	// Create initial state to go into queue
	SwarmState* initial = new SwarmState(positions, velocities, modes, 
		nearest_leaders, leader_countdowns, step_num, p.num_robots, 
		p.world_size);
	queue.push(initial);

	// Parameters for automated sequence planning
	float num_decisions = 5.0f;
	float segment_length = 600.0f;
	// Number of decisions in a trial
	float segments = static_cast<float>(p.step_limit) / segment_length;
	// Number of segments remaining to be simulated
	float s_r = powf(num_decisions, segments);

	// Number of times goal reached (only take the top 10)
	uint goals = 0;

	// Continue until queue has been exhausted
	while (queue.size() > 0) {
		// Get the top state on the queue
		SwarmState* s = queue.top();
		vector<Decision> b_seq(s->b_seq);
		Decision last_dec;
		if (b_seq.size() > 0) {
			last_dec = b_seq[b_seq.size() - 1];
		}
		queue.pop();

		// Check conditions for pruning a branch
		bool cond1 = b_seq.size() > 0 && s->connectivity < 0.001f;
		bool cond2 = b_seq.size() > 0 && s->heuristic < 90000.0f;
		float score_diff = 0.0f; bool cond3 = false;
		if (b_seq.size() > 0) {
			score_diff = last_dec.score_end - last_dec.score_start;
			cond3 = b_seq.size() > 0 && score_diff < 5000.0f;
		}

		// If this branch has reached the time limit, output behavior sequence; 
		// else, if this branch should be pruned, output the sequence; 
		// else, branch this behavior with the possible next behaviors
		if (s->step_num >= p.step_limit) {
			std::printf("Goal reached:\n");
			goal_reached = true;
			std::printf("Heuristic: %4.2f\n", s->heuristic);
			printDecisionSequence(b_seq);
			// Update high score
			if (last_dec.score_end > high_score) {
				high_score = last_dec.score_end;
				best_seq = b_seq;
			}
			goals++;

			// Break out of search if 10 goals reached
			//if (goals > 9) {
			//	break;
			//}
		}
		else if (b_seq.size() > 1 && (cond1 || cond2 || cond3)) {
			std::printf("Branch pruned on step %d due to:\n", s->step_num);
			if (cond1) {
				std::printf("loss of connectivity\n");
			}
			if (cond2) {
				std::printf("low heuristic value (%4.2f)\n", s->heuristic);
			}
			if (cond3) {
				std::printf("No progress on segment (%4.2f)\n", score_diff);
			}

			// Update segments remaining
			float pruned = powf(num_decisions, segments - 
				static_cast<float>(b_seq.size() + 1));
			s_r -= pruned;
		}
		else {
			// Temporary data variables for this branch
			float4* positions_t = (float4*)malloc(p.num_robots * sizeof(float4));
			float3* velocities_t = (float3*)malloc(p.num_robots * 
				sizeof(float3));
			int* modes_t = (int*)malloc(p.num_robots * sizeof(int));
			int* nearest_leaders_t = (int*)malloc(p.num_robots * sizeof(int));
			uint* leader_countdowns_t = (uint*)malloc(p.num_robots * 
				sizeof(uint));
			int* explored_grid_t = (int*)malloc(p.world_size * p.world_size * 
				sizeof(int));
			uint step_num_t = 0;

			// Copy robot data from popped state to temporary arrays
			for (uint i = 0; i < p.num_robots; i++) {
				positions_t[i] = s->pos[i];
				velocities_t[i] = s->vel[i];
				modes_t[i] = s->mode[i];
				nearest_leaders_t[i] = s->nearest_leader[i];
				leader_countdowns_t[i] = s->leader_countdown[i];
			}
			// Copy explored data from popped state to temporary grid
			for (uint i = 0; i < p.world_size * p.world_size; i++) {
				explored_grid_t[i] = s->explored[i];
			}

			// Get the step number of the popped state
			step_num_t = s->step_num;

			// If the previous behavior was flocking, allow branching to flock 
			// and disperse; else, branch only to flocking
			uint num_behaviors = static_cast<uint>(num_decisions);
			if (b_seq.size() > 0 && b_seq.at(b_seq.size() - 1).behavior == 2) {
				num_behaviors--;

				// Update segments remaining
				float pruned = powf(num_decisions, segments -
					static_cast<float>(b_seq.size()));
				s_r -= pruned;
			}
			// Branch the popped state with possible new behaviors
			for (uint i = 0; i < num_behaviors; i++) {

				// Set the simulation data on the GPU using the popped state
				setData(p.num_robots, positions_t, velocities_t, modes_t,
					nearest_leaders_t, leader_countdowns_t);
				step_num = step_num_t;

				// Set the explored grid data
				for (uint j = 0; j < p.world_size * p.world_size; j++){
					explored_grid[j] = explored_grid_t[j];
				}

				// Update the behavior sequence with next candidate behavior
				vector<Decision> b_seq_t(b_seq);
				Decision b;
				b.time_start = step_num;
				b.velocity = p.vel_bound;
				(b_seq.size() > 0) ? b.score_start = last_dec.score_end : 
					b.score_start = 0.0f;
				
				// Set simulation and decision point behavior based on branch num
				if (i != num_decisions - 1) {
					// Set to flock in NSEW direction
					p.behavior = 1;
					b.behavior = 1;
					goal_heading = -PI + (i * (PI / 2.0f)); // Four directions
					b.flock_dir = goal_heading;
					goal_vector = make_float3(cosf(goal_heading),
						-sinf(goal_heading), 0.0f);
				}
				else {
					// Set to dispersion
					p.behavior = 2;
					b.behavior = 2;
					b.flock_dir = 0.0f;
				}

				// Simulate 10 seconds with current behavior
				step(0);
				const clock_t start = clock();
				while ((step_num) % static_cast<uint>(segment_length) != 0) {
					step(0);
				}
				const clock_t end = clock();
				std::printf("Seg. completed at %d steps/sec ", 
					static_cast<uint>(1000.0f * (segment_length / 
					static_cast<float>(end - start))));

				// Update segments remaining
				s_r -= 1.0f;

				// Get the new score for this branch and update the behavior 
				// sequence
				b.time_end = step_num;
				b.score_end = data.score;
				b_seq_t.push_back(b);
				float heuristic = b.score_end / 
					static_cast<float>(b_seq_t.size());

				// Create the updated state with the results and put into queue
				SwarmState* s_new = new SwarmState(positions, velocities, modes,
					nearest_leaders, leader_countdowns, explored_grid, b_seq_t,
					step_num, data.score, heuristic, data.connectivity, 
					p.num_robots, p.world_size);
				queue.push(s_new);
				std::printf("(segments remaining = %1.0f)\n", s_r);
			}

			// Free arrays allocated in memory
			std::free(positions_t);
			std::free(velocities_t);
			std::free(modes_t);
			std::free(nearest_leaders_t);
			std::free(leader_countdowns_t);
			std::free(explored_grid_t);
		}

		// Delete old popped state
		delete s;
	}

	// Print out computation time
	std::printf("Best sequence algorithm completed in ");
	std::printf("%4.2f seconds\n",
		static_cast<float>(clock() - comp_time) / CLOCKS_PER_SEC, high_score);

	// Print and log best decision sequence if one reached the goal; 
	// else, output no goal reached
	if (goal_reached) {
		printDecisionSequence(best_seq);

		// Log this sequence to the world file if logging is on
		if (p.log_data) {
			for (uint i = 0; i < best_seq.size(); i++) {
				fprintf(world_f, "d %d %4.2f %4.2f %d %4.2f\n",
					best_seq[i].behavior, best_seq[i].flock_dir, 
					best_seq[i].velocity, best_seq[i].time_start, 
					best_seq[i].score_start);
			}
		}
	}
	else {
		std::printf("Goal not reached!\n");
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
				fopen_s(&output_f, output_fname.str().c_str(), "w");
				printDataHeader();
			}
			
			// Process data of initial state
			processData(p.num_robots, p.world_size, positions, velocities,
				explored_grid, laplacian, ap, &data);

			// Indicates inital state has passed
			initial_passed = true;
		}

		// If in playback mode (op_mode = 1), load next behavior if applicable
		if (p.op_mode == 1 && sequence.size() > 0 && 
			sequence[0].time_start == step_num) {

			// Set the correct behavior (flocking or disperse)
			p.vel_bound = sequence[0].velocity;
			switch (sequence[0].behavior) {
			case 1:		// Flocking
				p.behavior = 1;
				goal_heading = sequence[0].flock_dir;
				goal_vector = make_float3(cosf(goal_heading), 
					-sinf(goal_heading), 0.0f);
				break;
			case 2:		// Dispersion
				p.behavior = 2;
				break;
			}

			// Remove the behavior from sequence
			sequence.erase(sequence.begin(), sequence.begin() + 1);
		}

		// Launch the main kernel to perform one simulation step
		if (p.op_mode != 0) {
			launchMainKernel(goal_vector, step_num, leaders, ap, p, 
				&cuda_vbo_resource);
		}
		else {
			launchMainKernel(goal_vector, step_num, leaders, ap, p);
		}

		// Retrieve data from GPU (kernels.cu)
		getData(p.num_robots, positions, velocities, modes);
		// Update explored grid
		updateExplored();
		// Get data variables (data_ops.h)
		processData(p.num_robots, p.world_size, positions, velocities, 
			explored_grid, laplacian, ap, &data);

		if (p.log_data) {
			// Write data to the output log at the end of every step
			fprintf(output_f, "step %d %d %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f ",
				step_num, p.behavior, -goal_heading, p.vel_bound, 
				data.heading_avg, data.heading_var, data.centroid.x, 
				data.centroid.y);
			fprintf(output_f, "%4.2f %4.2f %d %d %d\n", data.ch_area, 
				data.connectivity, data.explored, data.targets_explored, 
				static_cast<int>(data.score));
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
	srand(static_cast<uint>(time(NULL)));

	///// PARAMETERS /////
	// If not in playback mode, load the parameters from file, given as first 
	// command line argument;
	// else, load the world file in playback mode
	loadParametersFromFile(argv[1]);
	if (p.op_mode == 1) {
		loadParametersFromFile(argv[2]);
	}
	// Begin paused if showing the GUI, otherwise start immediately
	if (p.op_mode != 0) {
		paused = true;
	}
	else {
		paused = false;
	}
	// Half the world size
	ws_2 = static_cast<float>(p.world_size) / 2.0f;
	// Open new data file for this trial
	output_fname << argv[3];

	///// MEMORY ALLOCATION /////
	explored_grid = (int*)malloc(p.world_size * p.world_size * sizeof(int));
	ap = (bool*)malloc(p.num_robots * sizeof(bool));
	targets = (int3*)malloc(p.targets * sizeof(int3));
	occupancy = (bool*)malloc(p.world_size * 10 * p.world_size * 10 * 
		sizeof(bool));
	// Initialize pinned host memory for data arrays
	cudaHostAlloc(&positions, p.num_robots * sizeof(float4), 0);
	cudaHostAlloc(&velocities, p.num_robots * sizeof(float3), 0);
	cudaHostAlloc(&modes, p.num_robots * sizeof(float4), 0);
	cudaHostAlloc(&leaders, p.num_robots * sizeof(int), 0);
	cudaHostAlloc(&nearest_leaders, p.num_robots * sizeof(int), 0);
	cudaHostAlloc(&leader_countdowns, p.num_robots * sizeof(uint), 0);
	cudaHostAlloc(&laplacian, p.num_robots * p.num_robots * sizeof(int4), 0);
	cudaHostAlloc(&obstacles, p.num_obstacles * sizeof(float4), 0);
	// Fill the leader list with -1 initially
	fill(leaders, leaders + p.num_robots, -1);
	// GPU memory allocation
	cudaAllocate(p);

	///// OPEN GL INITIALIZATION /////
	if (p.op_mode != 0) {
		// Initialize OpenGL
		initGL(argc, argv);
		// Create vertex buffer object (VBO)
		createVBO(&vbo_swarm, &cuda_vbo_resource, 
			cudaGraphicsMapFlagsWriteDiscard);
		// Set camera to default settings
		resetCamera();
	}

	///// CUDA INITIALIZATION /////
	if (p.op_mode != 0) {
		launchInitKernel(p, &cuda_vbo_resource);
	}
	else {
		launchInitKernel(p);
	}
	// Retrieve initial data from GPU (kernels.cu)
	getData(p.num_robots, positions, velocities, modes);
	getLaplacian(p.num_robots, laplacian);

	///// WORLD GENERATION OR LOADING /////
	// If not in playback mode (from saved world) generate obstacles; 
	// else, allocate and load obstacles in loadSavedMap function
	if (p.op_mode != 1) {
		generateWorld(argv[2]);
	}
	else {
		loadSavedMap(argv[2]);
		setData(p.num_robots, positions, velocities, modes);
	}

	///// OCCUPANCY AND EXPLORATION GRID INITIALIZATION /////
	calculateOccupancyGrid();
	// Send occupancy grid to GPU
	setOccupancy(p, occupancy);
	// Create the explored grid
	for (uint i = 0; i < p.world_size * p.world_size; i++) {
		// Get the coordinates of the grid cell
		float y = static_cast<float>(i % p.world_size) - ws_2;
		float x = floorf(static_cast<float>(i) / 
			static_cast<float>(p.world_size)) - ws_2;

		// Initialize cell to -1 if it is covered by an obstacle; 0 otherwise
		(checkCollision(x, y)) ? explored_grid[i] = -1 : explored_grid[i] = 0;
	}

	// Initialize goal heading to 0
	goal_heading = 0.0f;
	
	///// START MAIN LOOP /////
	if (p.op_mode == 0) {
		// Automate swarm control to find behavior sequence with best results
		automateExplore();
	}
	else {
		// Start the glut loop and show the display
		glutMainLoop();
	}

	///// QUIT /////
	exitSimulation();

	return 0;
}
