

const struct digit_tree
{
	//----------------------Left Leg
	double LHR_pos[3][1];
	double LHR_eul[1][3];
	double LHR_axis[1][3];

	double LHY_pos[3][1];
	double LHY_eul[1][3];
	double LHY_axis[1][3];

	double LHP_pos[3][1];
	double LHP_eul[1][3];
	double LHP_axis[1][3];

	double LK_pos[3][1];
	double LK_eul[1][3];
	double LK_axis[1][3];

	double LS_pos[3][1];
	double LS_eul[1][3];
	double LS_axis[1][3];

	double LT_pos[3][1];
	double LT_eul[1][3];
	double LT_axis[1][3];

	double LTP_pos[3][1];
	double LTP_eul[1][3];
	double LTP_axis[1][3];

	double LTR_pos[3][1];
	double LTR_eul[1][3];
	double LTR_axis[1][3];

	double LF_pos[3][1];
	double LF_eul[1][3];

	//Right leg -----------------------------------
	double RHR_pos[3][1];
	double RHR_eul[1][3];
	double RHR_axis[1][3];

	double RHY_pos[3][1];
	double RHY_eul[1][3];
	double RHY_axis[1][3];

	double RHP_pos[3][1];
	double RHP_eul[1][3];
	double RHP_axis[1][3];

	double RK_pos[3][1];
	double RK_eul[1][3];
	double RK_axis[1][3];

	double RS_pos[3][1];
	double RS_eul[1][3];
	double RS_axis[1][3];

	double RT_pos[3][1];
	double RT_eul[1][3];
	double RT_axis[1][3];

	double RTP_pos[3][1];
	double RTP_eul[1][3];
	double RTP_axis[1][3];

	double RTR_pos[3][1];
	double RTR_eul[1][3];
	double RTR_axis[1][3];

	double RF_pos[3][1];
	double RF_eul[1][3];

	//Left Arm ------------------------------------
	double LSR_pos[3][1];
	double LSR_eul[1][3];
	double LSR_axis[1][3];

	double LSP_pos[3][1];
	double LSP_eul[1][3];
	double LSP_axis[1][3];

	double LSY_pos[3][1];
	double LSY_eul[1][3];
	double LSY_axis[1][3];

	double LE_pos[3][1];
	double LE_eul[1][3];
	double LE_axis[1][3];

	//Right Arm ------------------------------------
	double RSR_pos[3][1];
	double RSR_eul[1][3];
	double RSR_axis[1][3];

	double RSP_pos[3][1];
	double RSP_eul[1][3];
	double RSP_axis[1][3];

	double RSY_pos[3][1];
	double RSY_eul[1][3];
	double RSY_axis[1][3];

	double RE_pos[3][1];
	double RE_eul[1][3];
	double RE_axis[1][3];



};

struct digit_tree tree = {
	//Left Leg -----------------------------------------------------
	{{-0.001},{0.091},{0}},                       //LHR_pos
	{0, -90*M_PI/180, -21.5*M_PI/180},            //LHR_eul
	{0, 0, 1},                                    //LHR_axis

	{{-0.0505},{0},{0.044}},                       //LHY_pos
	{0, -90*M_PI/180, 0},                          //LHY_eul
	{0, 0, 1},                                    //LHY_axiS

	{{0},{0.004},{0.068}},                       //LHP_pos
	{90*M_PI/180, 0, 135*M_PI/180},               //LHP_eul
	{0, 0, -1},                                    //LHP_axiS

	{{0.12},{0},{0.0045}},                       //LK_pos
	{0, 0, -90*M_PI/180},                          //LK_eul
	{0, 0, 1},                                    //LK_axiS

	{{0.060677},{0.047406},{0}},                       //LS_pos
	{0, 0, 0},                                      //LS_eul
	{0, 0, 1},                                    //LS_axiS

	{{0.434759},{0.02},{0}},                       //LT_pos
	{0, 0, 103*M_PI/180},                          //LT_eul
	{0, 0, 1},                                    //LT_axiS

	{{0.408},{-0.04},{0}},                       //LTP_pos
	{0, 0, 68.5*M_PI/180},                          //LTP_eul
	{0, 0, 1},                                    //LTP_axiS

	{{0},{0},{0}},                                //LTR_pos
	{0, 90*M_PI/180, 0},                          //LTR_eul
	{0, 0, 1},                                    //LTR_axiS

	{{0},{-0.05456},{-0.0315}},                      //LF_pos
	{-60*M_PI/180, 0, -90*M_PI/180},                 //LF_eul

	//Right Leg -------------------------------------------------------------

	{{-0.001},{-0.091},{0}},                       //RHR_pos
	{0, -90*M_PI/180, 21.5*M_PI/180},            //RHR_eul
	{0, 0, 1},                                    //RHR_axis

	{{-0.0505},{0},{0.044}},                       //RHY_pos
	{0, -90*M_PI/180, 0},                          //RHY_eul
	{0, 0, 1},                                    //RHY_axiS

	{{0},{-0.004},{0.068}},                       //RHP_pos
	{-90*M_PI/180, 0, -135*M_PI/180},               //RHP_eul
	{0, 0, -1},                                    //RHP_axiS

	{{0.12},{0},{0.0045}},                       //RK_pos
	{0, 0, 90*M_PI/180},                          //RK_eul
	{0, 0, 1},                                    //RK_axiS

	{{0.060677},{-0.047406},{0}},                       //RS_pos
	{0, 0, 0},                                      //RS_eul
	{0, 0, 1},                                    //RS_axiS

	{{0.434759},{-0.02},{0}},                       //RT_pos
	{0, 0, -103*M_PI/180},                          //RT_eul
	{0, 0, 1},                                    //RT_axiS

	{{0.408},{0.04},{0}},                       //RTP_pos
	{0, 0, -68.5*M_PI/180},                          //RTP_eul
	{0, 0, 1},                                    //RTP_axiS

	{{0},{0},{0}},                                //RTR_pos
	{0, 90*M_PI/180, 0},                          //RTR_eul
	{0, 0, 1},                                    //RTR_axiS

	{{0},{0.05456},{-0.0315}},                      //RF_pos
	{60*M_PI/180, 0, 90*M_PI/180},                 //RF_eul

	//Left Arm -------------------------------------------------------------------
	{{-0.001},{0.12},{0.4}},                       //LSR_pos
	{-10*M_PI/180, -90*M_PI/180, 0},               //LSR_eul
	{0, 0, 1},                                    //LSR_axiS

	{{-0.00317},{-0.011055},{0.0555}},            //LSP_pos
	{90*M_PI/180, -16*M_PI/180, -45*M_PI/180},    //LSP_eul
	{0, 0, -1},                                    //LSP_axiS

	{{0},{-0.165},{-0.1}},                       //LSY_pos
	{90*M_PI/180,0,0},                          //LSY_eul
	{0, 0, 1},                                    //LSY_axiS

	{{0},{-0.0385},{0.185}},                      //LE_pos
	{90*M_PI/180, 0, 22.5*M_PI/180},              //LE_eul
	{0, 0, 1},                                    //LE_axiS

	//Right Arm -------------------------------------------------------------------
	{{-0.001},{-0.12},{0.4}},                       //RSR_pos
	{10*M_PI/180, -90*M_PI/180, 0},               //RSR_eul
	{0, 0, 1},                                    //RSR_axiS

	{{-0.00317},{0.011055},{0.0555}},            //RSP_pos
	{-90*M_PI/180, -16*M_PI/180, 45*M_PI/180},    //RSP_eul
	{0, 0, -1},                                    //RSP_axiS

	{{0},{0.165},{-0.1}},                       //RSY_pos
	{-90*M_PI/180,0,0},                          //RSY_eul
	{0, 0, 1},                                    //RSY_axiS

	{{0},{0.0385},{0.185}},                      //RE_pos
	{-90*M_PI/180, 0, -22.5*M_PI/180},              //RE_eul
	{0, 0, 1},                                    //RE_axiS

};


