// sfun_FlightGearSender.c
// S-Function interface for FlightGear Flight Simulator

// NOTE: To integrate this into a Simulink Model, use the .m script called "make_FlightGearSender.m"

#define S_FUNCTION_NAME  sfun_FlightGearSender
#define S_FUNCTION_LEVEL 2

// This is for 32bit versus 64bit Operating Systems. Most are 64 bit now, and that is what is used below unless you comment things out
//#pragma comment(lib,"WS2_32.Lib")
#pragma comment(lib,"WS2_32_x64.Lib")

// Windows Socket Utilities for UDP communications
#include <winsock2.h>

// Matlab/Simulink structures and utilities, this comes with matlab and the .m script will find it I think in default matlab paths
#include "simstruc.h"

// Flight Gear data structure, needed for the structures defined below
#include "net_fdm.hxx"

// S-function parameters, things done for every Sfunction
#define NUM_PARAMS (3)
#define TS_PARAM (ssGetSFcnParam(S,0))
#define HOSTNAME_PARAM (ssGetSFcnParam(S,1))
#define PORT_PARAM (ssGetSFcnParam(S,2))

// Macros to access the S-function parameter values
#define SAMPLE_TIME (mxGetPr(TS_PARAM)[0])

// Seems to be that Windows htond does not swap bytes correctly (for FlightGear).
//   This seems to work.
static void htond_local (double &x)	
{
    int    *Double_Overlay;
    int     Holding_Buffer;
   
    Double_Overlay = (int *) &x;
    Holding_Buffer = Double_Overlay [0];
   
    Double_Overlay [0] = htonl (Double_Overlay [1]);
    Double_Overlay [1] = htonl (Holding_Buffer);

}

// Since carrying my own htond, might as well bring along an htonf as well
void htonf_local (float* x)	
{
    int    *Float_Overlay;
    int     Holding_Buffer;
    
    Float_Overlay = (int *) x;
    Holding_Buffer = Float_Overlay [0];
    
    Float_Overlay [0] = htonl (Holding_Buffer);
}

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    mexPrintf("sfun_FlightGearSender-InitializingSizes\n");

    /* Set Up data structure for WinSock class */
    WSADATA      wsaData;

    ssSetNumSFcnParams(S, NUM_PARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 2)) return;

	// Position input
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 0, 1);

	// Attitude input
    ssSetInputPortWidth(S, 1, 3);
    ssSetInputPortRequiredContiguous(S, 1, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 1, 1);

	// There is no numerical output from this function
    if (!ssSetNumOutputPorts(S, 0)) return;

	ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 2);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, 0);

	// Initialize sockets
	WSAStartup( 0x101,&wsaData );
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    mexPrintf("sfun_FlightGearSender-InitializingSampleTimes\n");

    ssSetSampleTime(S, 0, SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#undef MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
      mexPrintf("sfun_FlightGearSender-Starting\n");
      FILE* pFile;
      pFile=fopen("log_debug_FlightGearSender.txt","a+");
      fprintf(pFile,"Started \n");
      struct sockaddr_in sa;
      struct hostent     *hp;
      char HostName[256];
      int buflen;
      SOCKET* pFGSocket;
      unsigned int Port = 5502;
      BOOL FGConnected = FALSE;
      void **PWork = ssGetPWork(S);
      
      // Allocate memory for persistent variables
      PWork[0] = calloc(1, sizeof(SOCKET));
      PWork[1] = calloc(1, sizeof(FGNetFDM));
      
      // The FGSocket pointer points to one of the persistent variables
      pFGSocket = (SOCKET*)PWork[0];
      
      // Initialize the persistent variables
      memset(PWork[0], 0, sizeof(SOCKET));
      memset(PWork[1], 0, sizeof(FGNetFDM));
      
      // Read the host name parameter
      buflen = 256;
      mxGetString(HOSTNAME_PARAM, HostName, buflen);
      fprintf(pFile,"HostName: %s \n",HostName);
      
      // Read the port number
      Port = (unsigned int)(mxGetPr(PORT_PARAM)[0]);
      fprintf(pFile,"Port: %i \n",Port);
      
      // Connect socket
      if((hp= gethostbyname(HostName)) != NULL)
      {
          fprintf(pFile,"Found Host\n");
          // Clear address data structure
          memset(&sa,0,sizeof(sa));
          
          // Copy the address data from the host name lookup
          memcpy((char *)&sa.sin_addr,hp->h_addr,hp->h_length);     /* set address */
          
          // Set the address family (should be AF_INET)
          sa.sin_family= hp->h_addrtype;
          
          // Set the port number
          sa.sin_port= htons((u_short)Port);
          
          // Create a datagram socket
          *pFGSocket = socket(hp->h_addrtype,SOCK_DGRAM,0);
          
          if(*pFGSocket >= 0)
          {
              fprintf(pFile,"pFGSocket>=0 :: Socket Created\n");
              // Connect the socket, should always succeed for UDP
              if(connect(*pFGSocket,(struct sockaddr *)&sa, sizeof(sa)) < 0)
              {
                  fprintf(pFile,"Socket Failed to Connect. Closing Socket\n");
                  closesocket(*pFGSocket);	// Uh-oh, problem
              }// If failed to connect socket
              else
              {
                  fprintf(pFile,"Socket Connected\n");
                  // Set the socket to non-blocking mode
                  u_long arg = 1;
                  if (ioctlsocket(*pFGSocket, FIONBIO, &arg ) == 0)
                  {
                      fprintf(pFile,"FG Connected\n");
                      FGConnected = TRUE;
                  }
                  else
                  {
                      fprintf(pFile,"FG Not Connected. Closing Socket.\n");
                      closesocket(*pFGSocket);
                  }
              }// If socket connected
          }// If created a socket
      }// If got FG computer address
      
      // Exit if not connected
      if(!FGConnected)
      {
          fprintf(pFile,"!FG Connected. Set Error State.\n");
          ssSetErrorStatus(S,"Could not connect to host !");
          return;
      }
      // The next section will send an badly formatted message to Flight Gear, which will cause it to crash.
      // This can be useful at times to see what the Flight Gear log
      /*if (send(*pFGSocket, (const char*)HostName, sizeof(HostName), 0) < 0) {
        fprintf(pFile,"!Send Failed. \n");
    }
    else {
        fprintf(pFile,"!Send Succeeded. \n");
    }*/
      
      
      fclose(pFile);
      mexPrintf("sfun_FlightGearSender-Started\n");
  }
#endif /*  MDL_START */

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector, ssGetY(S).
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T *posvec = (const real_T*) ssGetInputPortSignal(S,0);
	const real_T *attvec = (const real_T*) ssGetInputPortSignal(S,1);
	void **PWork = ssGetPWork(S);
	SOCKET* pFGSocket = (SOCKET*)PWork[0];
	FGNetFDM* pFGNet = (FGNetFDM*)PWork[1];

	// Position
	pFGNet->latitude =  posvec[0];
	pFGNet->longitude = posvec[1];
	pFGNet->altitude = posvec[2];

	// Attitude
	pFGNet->phi = attvec[0];
	pFGNet->theta = attvec[1];
	pFGNet->psi = attvec[2];

    htond_local(pFGNet->latitude);
    htond_local(pFGNet->longitude);
    htond_local(pFGNet->altitude);
    htonf_local(&pFGNet->phi);
    htonf_local(&pFGNet->theta);
    htonf_local(&pFGNet->psi);

    pFGNet->version = FG_NET_FDM_VERSION;
    pFGNet->version = htonl(pFGNet->version);
    
    // These can be set if desired, but no need.
/*    pFGNet->agl = 0;
    htonf_local(&pFGNet->agl);

    pFGNet->alpha = 0;
    pFGNet->beta = 0;
    htonf_local(&pFGNet->alpha);
    htonf_local(&pFGNet->beta);
    
    pFGNet->phidot=0;       // roll rate (radians/sec)
    pFGNet->thetadot=0;     // pitch rate (radians/sec)
    pFGNet->psidot=0;       // yaw rate (radians/sec)
    pFGNet->vcas=0;             // calibrated airspeed
    pFGNet->climb_rate=0;       // feet per second
    pFGNet->v_north=0;              // north velocity in local/body frame, fps
    pFGNet->v_east=0;               // east velocity in local/body frame, fps
    pFGNet->v_down=0;               // down/vertical velocity in local/body frame, fps
    pFGNet->v_body_u=0;    // ECEF velocity in body frame
    pFGNet->v_body_v=0;    // ECEF velocity in body frame 
    pFGNet->v_body_w=0;    // ECEF velocity in body frame
    htonf_local(&pFGNet->phidot);
    htonf_local(&pFGNet->thetadot);
    htonf_local(&pFGNet->psidot);
    htonf_local(&pFGNet->vcas);
    htonf_local(&pFGNet->climb_rate);
    htonf_local(&pFGNet->v_north);
    htonf_local(&pFGNet->v_east);
    htonf_local(&pFGNet->v_down);
    htonf_local(&pFGNet->v_body_u);
    htonf_local(&pFGNet->v_body_v);
    htonf_local(&pFGNet->v_body_w);

    pFGNet->visibility=10000;
    htonf_local(&pFGNet->visibility);            // visibility in meters (for env. effects)
    
    pFGNet->A_X_pilot;        // X accel in body frame ft/sec^2
    pFGNet->A_Y_pilot;        // Y accel in body frame ft/sec^2
    pFGNet->A_Z_pilot;        // Z accel in body frame ft/sec^2

    // Stall
    pFGNet->stall_warning;        // 0.0 - 1.0 indicating the amount of stall
    pFGNet->slip_deg;     // slip ball deflection

    // Pressure
    
    // Engine status
    pFGNet->num_engines;        // Number of valid engines
    pFGNet->eng_state[FG_MAX_ENGINES];// Engine state (off, cranking, running)
    pFGNet->rpm[FG_MAX_ENGINES];       // Engine RPM rev/min
    pFGNet->fuel_flow[FG_MAX_ENGINES]; // Fuel flow gallons/hr
    pFGNet->fuel_px[FG_MAX_ENGINES];   // Fuel pressure psi
    pFGNet->egt[FG_MAX_ENGINES];       // Exhuast gas temp deg F
    pFGNet->cht[FG_MAX_ENGINES];       // Cylinder head temp deg F
    pFGNet->mp_osi[FG_MAX_ENGINES];    // Manifold pressure
    pFGNet->tit[FG_MAX_ENGINES];       // Turbine Inlet Temperature
    pFGNet->oil_temp[FG_MAX_ENGINES];  // Oil temp deg F
    pFGNet->oil_px[FG_MAX_ENGINES];    // Oil pressure psi

    // Consumables
    pFGNet->num_tanks;     // Max number of fuel tanks
    pFGNet->fuel_quantity[FG_MAX_TANKS];

    // Gear status
    pFGNet->num_wheels;
    pFGNet->wow[FG_MAX_WHEELS];
    pFGNet->gear_pos[FG_MAX_WHEELS];
    pFGNet->gear_steer[FG_MAX_WHEELS];
    pFGNet->gear_compression[FG_MAX_WHEELS];

    // Environment
    pFGNet->cur_time;           // current unix time
                                 // FIXME: make this uint64_t before 2038
    pFGNet->warp;                // offset in seconds to unix time

    // Control surface positions (normalized values)
    pFGNet->elevator;
    pFGNet->elevator_trim_tab;
    pFGNet->left_flap;
    pFGNet->right_flap;
    pFGNet->left_aileron;
    pFGNet->right_aileron;
    pFGNet->rudder;
    pFGNet->nose_wheel;
    pFGNet->speedbrake;
    pFGNet->spoilers;	
*/	
	// Send to FlightGear
	if (send(*pFGSocket, (const char*)pFGNet, sizeof(FGNetFDM), 0) < 0) {
        mexPrintf("Failed to send");
    }    
}



#undef MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#undef MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
	void **PWork = ssGetPWork(S);
	SOCKET* pFGSocket = (SOCKET*)PWork[0];
	
	// Close the socket connection
	closesocket(*pFGSocket);

	// Free persistent variables
	free(PWork[0]);
	free(PWork[1]);
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
