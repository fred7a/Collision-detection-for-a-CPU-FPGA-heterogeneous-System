/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith. Alves, Fredy.       *
 * All rights reserved.  Email: russ@q12.org fredyamalves1@gmail.com   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this software in the    *
 *       file LICENSE.                                                   *
 *                                                                       *
 * This software is distributed in the hope that it will be useful,      *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the file     *
 * LICENSE for more details.                     						 *
 *                                                                       *
 *************************************************************************/


#include <aalsdk/AAL.h>

#include <aalsdk/xlRuntime.h>

//#include <aalsdk/AALLoggerExtern.h> // Logger





#include <aalsdk/service/ISPLAFU.h>       // Service Interface

#include <aalsdk/service/ISPLClient.h>    // Service Client Interface

#include <aalsdk/kernel/vafu2defs.h>      // AFU structure definitions (brings in spl2defs.h)



#include <string.h>

#include <cassert>



#include <ode/ode.h>

#include <drawstuff/drawstuff.h>
#include <iostream>
#include <list>
#include <math.h>       /* pow */

#include "texturepath.h"

#include <../ode/src/collision_util.h>

#include <../ode/src/collision_std.h>



#ifdef _MSC_VER

#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints

#endif



#include "icosahedron_geom.h"



#include <sstream>

#include <iomanip>

#include <stdio.h>

#include <ctime>

#include <vector>

#include <chrono>

#include <sys/time.h>

//#include <chrono>



//****************************************************************************

// UN-COMMENT appropriate #define in order to enable either Hardware or ASE.

//    DEFAULT is to use Software Simulation.

//****************************************************************************

 #define  HWAFU

//#define  ASEAFU



using namespace AAL;

using namespace std::chrono;



#define CSR_CTL                  0x1a2c



// Convenience macros for printing messages and errors.

#ifdef MSG

# undef MSG

#endif // MSG

#define MSG(x) std::cout << __AAL_SHORT_FILE__ << ':' << __LINE__ << ':' << __AAL_FUNC__ << "() : " << x << std::endl

#ifdef ERR

# undef ERR

#endif // ERR

# define ERR(x) std::cerr << __AAL_SHORT_FILE__ << ':' << __LINE__ << ':' << __AAL_FUNC__ << "() **Error : " << x << std::endl



// Print/don't print the event ID's entered in the event handlers.

#if 1

# define EVENT_CASE(x) case x : MSG(#x);

#else

# define EVENT_CASE(x) case x :

#endif



#ifndef CL

# define CL(x)                     ((x) * 64)

#endif // CL

#ifndef LOG2_CL

# define LOG2_CL                   6

#endif // LOG2_CL

#ifndef MB

# define MB(x)                     ((x) * 1024 * 1024)

#endif // MB



#define LPBK1_DSM_SIZE           MB(4)



/// @addtogroup SudokuSample

/// @{



//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################

btVirtAddr         pWSUsrVirt;
btWSSize     WSLen;


// Number of bytes in each of the source and destination buffers (4 MiB in this case)
btUnsigned32bitInt a_num_bytes;
btUnsigned32bitInt a_num_cl;

// VAFU Context is at the beginning of the buffer
VAFU2_CNTXT       *pVAFU2_cntxt;

// The source buffer is right after the VAFU Context
btVirtAddr         pSource;

// The destination buffer is right after the source buffer
btVirtAddr         pDest;

struct OneCL {                      // Make a cache-line sized structure
btUnsigned32bitInt dw[16];       //    for array arithmetic
};
struct OneCL      *pSourceCL;
struct OneCL      *pDestCL;
	


static void nearCallback (void *data, dGeomID o1, dGeomID o2);

static void start();

static void simLoop (int pause);

// dynamics and collision objects

static dWorldID world;
static dSpaceID space;

static dMass m;
static dJointGroupID contactgroup;

std::list<dGeomID> spheresList;

struct collisionInfo {
  float x1;  
  float x2;
  float x3;
  float r1;
  float y1;  
  float y2;
  float y3;
  float r2;
};
std::list<collisionInfo> colInfosList;


#define FORCE
#define GRAPH



//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################
//####################################################### GLOBAL VARIABLES ###################################################################################




/// @brief   Define our Runtime client class so that we can receive the runtime started/stopped notifications.

///

/// We implement a Service client within, to handle AAL Service allocation/free.

/// We also implement a Semaphore for synchronization with the AAL runtime.

class RuntimeClient : public CAASBase,

                      public IRuntimeClient

{

public:

   RuntimeClient();

   ~RuntimeClient();



   void end();



   IRuntime* getRuntime();



   btBool isOK();



   // <begin IRuntimeClient interface>

   void runtimeStarted(IRuntime            *pRuntime,

                       const NamedValueSet &rConfigParms);





   void runtimeStopped(IRuntime *pRuntime);



   void runtimeStartFailed(const IEvent &rEvent);



   void runtimeAllocateServiceFailed( IEvent const &rEvent);



   void runtimeAllocateServiceSucceeded(IBase               *pClient,

                                        TransactionID const &rTranID);



   void runtimeEvent(const IEvent &rEvent);





   // <end IRuntimeClient interface>





protected:

   IRuntime        *m_pRuntime;  // Pointer to AAL runtime instance.

   Runtime          m_Runtime;   // AAL Runtime

   btBool           m_isOK;      // Status

   CSemaphore       m_Sem;       // For synchronizing with the AAL runtime.

};



///////////////////////////////////////////////////////////////////////////////

///

///  MyRuntimeClient Implementation

///

///////////////////////////////////////////////////////////////////////////////

RuntimeClient::RuntimeClient() :

    m_Runtime(),        // Instantiate the AAL Runtime

    m_pRuntime(NULL),

    m_isOK(false)

{

   NamedValueSet configArgs;

   NamedValueSet configRecord;



   // Publish our interface

   SetSubClassInterface(iidRuntimeClient, dynamic_cast<IRuntimeClient *>(this));



   m_Sem.Create(0, 1);



   // Using Hardware Services requires the Remote Resource Manager Broker Service

   //  Note that this could also be accomplished by setting the environment variable

   //   XLRUNTIME_CONFIG_BROKER_SERVICE to librrmbroker

#if defined( HWAFU )

   configRecord.Add(XLRUNTIME_CONFIG_BROKER_SERVICE, "librrmbroker");

   configArgs.Add(XLRUNTIME_CONFIG_RECORD,configRecord);

#endif



   if(!m_Runtime.start(this, configArgs)){

      m_isOK = false;

      return;

   }

   m_Sem.Wait();

}



RuntimeClient::~RuntimeClient()

{

    m_Sem.Destroy();

}



btBool RuntimeClient::isOK()

{

   return m_isOK;

}



void RuntimeClient::runtimeStarted(IRuntime *pRuntime,

                                   const NamedValueSet &rConfigParms)

{

   // Save a copy of our runtime interface instance.

   m_pRuntime = pRuntime;

   m_isOK = true;

   m_Sem.Post(1);

}



void RuntimeClient::end()

{

   m_Runtime.stop();

   m_Sem.Wait();

}



void RuntimeClient::runtimeStopped(IRuntime *pRuntime)

{

   //MSG("Runtime stopped");

   m_isOK = false;

   m_Sem.Post(1);

}



void RuntimeClient::runtimeStartFailed(const IEvent &rEvent)

{

   IExceptionTransactionEvent * pExEvent = dynamic_ptr<IExceptionTransactionEvent>(iidExTranEvent, rEvent);

   ERR("Runtime start failed");

   ERR(pExEvent->Description());

}



void RuntimeClient::runtimeAllocateServiceFailed(IEvent const &rEvent)

{

   IExceptionTransactionEvent * pExEvent = dynamic_ptr<IExceptionTransactionEvent>(iidExTranEvent, rEvent);

   ERR("Runtime AllocateService failed");

   ERR(pExEvent->Description());

}



void RuntimeClient::runtimeAllocateServiceSucceeded(IBase *pClient,

                                                    TransactionID const &rTranID)

{

   //MSG("Runtime Allocate Service Succeeded");

}



void RuntimeClient::runtimeEvent(const IEvent &rEvent)

{

   //MSG("Generic message handler (runtime)");

}



IRuntime * RuntimeClient::getRuntime()

{

   return m_pRuntime;

}





/// @brief   Define our Service client class so that we can receive Service-related notifications from the AAL Runtime.

///          The Service Client contains the application logic.

///

/// When we request an AFU (Service) from AAL, the request will be fulfilled by calling into this interface.

class SphereCollision: public CAASBase, public IServiceClient, public ISPLClient

{

public:



   SphereCollision(RuntimeClient * rtc, char *puzName);

   ~SphereCollision();



   btInt run(uint32_t value_times);



   // <ISPLClient>

   virtual void OnTransactionStarted(TransactionID const &TranID,

                                     btVirtAddr AFUDSM,

                                     btWSSize AFUDSMSize);

   virtual void OnContextWorkspaceSet(TransactionID const &TranID);



   virtual void OnTransactionFailed(const IEvent &Event);



   virtual void OnTransactionComplete(TransactionID const &TranID);



   virtual void OnTransactionStopped(TransactionID const &TranID);

   virtual void OnWorkspaceAllocated(TransactionID const &TranID,

                                     btVirtAddr WkspcVirt,

                                     btPhysAddr WkspcPhys,

                                     btWSSize WkspcSize);



   virtual void OnWorkspaceAllocateFailed(const IEvent &Event);



   virtual void OnWorkspaceFreed(TransactionID const &TranID);



   virtual void OnWorkspaceFreeFailed(const IEvent &Event);

   // </ISPLClient>



   // <begin IServiceClient interface>

   virtual void serviceAllocated(IBase *pServiceBase,

                                 TransactionID const &rTranID);



   virtual void serviceAllocateFailed(const IEvent &rEvent);



   virtual void serviceFreed(TransactionID const &rTranID);



   virtual void serviceEvent(const IEvent &rEvent);

   // <end IServiceClient interface>

   static int dCollideSphereSphereAAL (dxGeom *o1, dxGeom *o2, int flags,

                          dContactGeom *contact, int skip);


   void initialize();
   void startTransaction(uint32_t value_times);

void finish();

   

   protected:

       

   



   char          *m_puzName;

   IBase         *m_pAALService;    // The generic AAL Service interface for the AFU.

   RuntimeClient *m_runtimClient;

   ISPLAFU       *m_SPLService;

   CSemaphore     m_Sem;            // For synchronizing with the AAL runtime.

   btInt          m_Result;



   // Workspace info

   btVirtAddr     m_pWkspcVirt;     ///< Workspace virtual address.

   btWSSize       m_WkspcSize;      ///< DSM workspace size in bytes.



   btVirtAddr     m_AFUDSMVirt;     ///< Points to DSM

   btWSSize       m_AFUDSMSize;     ///< Length in bytes of DSM



};









/* DBS: for sudoku */











///////////////////////////////////////////////////////////////////////////////

///

///  Implementation

///

///////////////////////////////////////////////////////////////////////////////

SphereCollision::SphereCollision(RuntimeClient *rtc, char *puzName) :

   m_puzName(puzName),

   m_pAALService(NULL),

   m_runtimClient(rtc),

   m_SPLService(NULL),

   m_Result(0),

   m_pWkspcVirt(NULL),

   m_WkspcSize(0),

   m_AFUDSMVirt(NULL),

   m_AFUDSMSize(0)

{

   SetSubClassInterface(iidServiceClient, dynamic_cast<IServiceClient *>(this));

   SetInterface(iidSPLClient, dynamic_cast<ISPLClient *>(this));

   SetInterface(iidCCIClient, dynamic_cast<ICCIClient *>(this));

   m_Sem.Create(0, 1);

}







SphereCollision::~SphereCollision()

{

   m_Sem.Destroy();

}

// We must implement the IServiceClient interface (IServiceClient.h):



// <begin IServiceClient interface>

void SphereCollision::serviceAllocated(IBase               *pServiceBase,

                              TransactionID const &rTranID)

{

   m_pAALService = pServiceBase;

   ASSERT(NULL != m_pAALService);



   // Documentation says SPLAFU Service publishes ISPLAFU as subclass interface

   m_SPLService = subclass_ptr<ISPLAFU>(pServiceBase);



   ASSERT(NULL != m_SPLService);

   if ( NULL == m_SPLService ) {

      return;

   }



   //MSG("Service Allocated");



   // Allocate Workspaces needed. ASE runs more slowly and we want to watch the transfers,

   //   so have fewer of them.

#if defined ( ASEAFU )

#define LB_BUFFER_SIZE CL(1000)

#else

#define LB_BUFFER_SIZE MB(4)

#endif



   m_SPLService->WorkspaceAllocate(sizeof(VAFU2_CNTXT) + LB_BUFFER_SIZE + LB_BUFFER_SIZE,

      TransactionID());



}



void SphereCollision::serviceAllocateFailed(const IEvent &rEvent)

{

   IExceptionTransactionEvent * pExEvent = dynamic_ptr<IExceptionTransactionEvent>(iidExTranEvent, rEvent);

   ERR("Failed to allocate a Service");

   ERR(pExEvent->Description());

   ++m_Result;

   m_Sem.Post(1);

}




void SphereCollision::serviceFreed(TransactionID const &rTranID)

{

   //MSG("Service Freed");

   // Unblock Main()

   m_Sem.Post(1);

}



// <ISPLClient>

void SphereCollision::OnWorkspaceAllocated(TransactionID const &TranID,

                                  btVirtAddr           WkspcVirt,

                                  btPhysAddr           WkspcPhys,

                                  btWSSize             WkspcSize)

{

   AutoLock(this);



   m_pWkspcVirt = WkspcVirt;

   m_WkspcSize = WkspcSize;



   //INFO("Got Workspace");         // Got workspace so unblock the Run() thread

   m_Sem.Post(1);

}



void SphereCollision::OnWorkspaceAllocateFailed(const IEvent &rEvent)

{

   IExceptionTransactionEvent * pExEvent = dynamic_ptr<IExceptionTransactionEvent>(iidExTranEvent, rEvent);

   ERR("OnWorkspaceAllocateFailed");

   ERR(pExEvent->Description());

   ++m_Result;

   m_Sem.Post(1);

}



void SphereCollision::OnWorkspaceFreed(TransactionID const &TranID)

{

  // ERR("OnWorkspaceFreed");

   // Freed so now Release() the Service through the Services IAALService::Release() method

   (dynamic_ptr<IAALService>(iidService, m_pAALService))->Release(TransactionID());

}



void SphereCollision::OnWorkspaceFreeFailed(const IEvent &rEvent)

{

   IExceptionTransactionEvent * pExEvent = dynamic_ptr<IExceptionTransactionEvent>(iidExTranEvent, rEvent);

   ERR("OnWorkspaceAllocateFailed");

   ERR(pExEvent->Description());

   ++m_Result;

   m_Sem.Post(1);

}



/// CMyApp Client implementation of ISPLClient::OnTransactionStarted

void SphereCollision::OnTransactionStarted( TransactionID const &TranID,

                                   btVirtAddr           AFUDSMVirt,

                                   btWSSize             AFUDSMSize)

{

   //INFO("Transaction Started");

   m_AFUDSMVirt = AFUDSMVirt;

   m_AFUDSMSize =  AFUDSMSize;

   m_Sem.Post(1);

}

/// CMyApp Client implementation of ISPLClient::OnContextWorkspaceSet

void SphereCollision::OnContextWorkspaceSet( TransactionID const &TranID)

{

   //INFO("Context Set");

   m_Sem.Post(1);

}

/// CMyApp Client implementation of ISPLClient::OnTransactionFailed

void SphereCollision::OnTransactionFailed( const IEvent &rEvent)

{

   IExceptionTransactionEvent * pExEvent = dynamic_ptr<IExceptionTransactionEvent>(iidExTranEvent, rEvent);

   //MSG("Runtime AllocateService failed");

   //MSG(pExEvent->Description());

   m_bIsOK = false;

   ++m_Result;

   m_AFUDSMVirt = NULL;

   m_AFUDSMSize =  0;

   ERR("Transaction Failed");

   m_Sem.Post(1);

}

/// CMyApp Client implementation of ISPLClient::OnTransactionComplete

void SphereCollision::OnTransactionComplete( TransactionID const &TranID)

{

   m_AFUDSMVirt = NULL;

   m_AFUDSMSize =  0;

   //INFO("Transaction Complete");

   m_Sem.Post(1);

}

/// CMyApp Client implementation of ISPLClient::OnTransactionStopped

void SphereCollision::OnTransactionStopped( TransactionID const &TranID)

{

   m_AFUDSMVirt = NULL;

   m_AFUDSMSize =  0;

  // INFO("Transaction Stopped");

   m_Sem.Post(1);

}

void SphereCollision::serviceEvent(const IEvent &rEvent)

{

   ERR("unexpected event 0x" << hex << rEvent.SubClassID());

}


void SphereCollision::initialize(){
       cout <<"======================="<<endl;
   cout <<"= Hello SPL LB Sample ="<<endl;
   cout <<"======================="<<endl;

   // Request our AFU.

   // NOTE: This example is bypassing the Resource Manager's configuration record lookup
   //  mechanism.  This code is work around code and subject to change.
   NamedValueSet Manifest;
   NamedValueSet ConfigRecord;


#if defined( HWAFU )                /* Use FPGA hardware */
   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_SERVICE_NAME, "libHWSPLAFU");
   ConfigRecord.Add(keyRegAFU_ID,"5DA62813-9A75-4228-8FDB-5D4006DD55CE");
   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_AIA_NAME, "libAASUAIA");

   #elif defined ( ASEAFU )
   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_SERVICE_NAME, "libASESPLAFU");
   ConfigRecord.Add(AAL_FACTORY_CREATE_SOFTWARE_SERVICE,true);

#else

   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_SERVICE_NAME, "libSWSimSPLAFU");
   ConfigRecord.Add(AAL_FACTORY_CREATE_SOFTWARE_SERVICE,true);
#endif

   Manifest.Add(AAL_FACTORY_CREATE_CONFIGRECORD_INCLUDED, ConfigRecord);

   Manifest.Add(AAL_FACTORY_CREATE_SERVICENAME, "Hello SPL LB");

   MSG("Allocating Service");

   // Allocate the Service and allocate the required workspace.
   //   This happens in the background via callbacks (simple state machine).
   //   When everything is set we do the real work here in the main thread.
   m_runtimClient->getRuntime()->allocService(dynamic_cast<IBase *>(this), Manifest);

   m_Sem.Wait();

   // If all went well run test.
   //   NOTE: If not successful we simply bail.
   //         A better design would do all appropriate clean-up.
   if(0 == m_Result){


	//=============================
	// Now we have the NLB Service
	//   now we can use it
	//=============================
	MSG("Running Test");
	
	pWSUsrVirt = m_pWkspcVirt; // Address of Workspace
	WSLen      = m_WkspcSize; // Length of workspace
	
	INFO("Allocated " << WSLen << "-byte Workspace at virtual address "
					<< std::hex << (void *)pWSUsrVirt);
	
	// Number of bytes in each of the source and destination buffers (4 MiB in this case)
	a_num_bytes= (btUnsigned32bitInt) ((WSLen - sizeof(VAFU2_CNTXT)) / 2);
	a_num_cl   = a_num_bytes / CL(1);  // number of cache lines in buffer
	
	// VAFU Context is at the beginning of the buffer
	pVAFU2_cntxt = reinterpret_cast<VAFU2_CNTXT *>(pWSUsrVirt);
	
	// The source buffer is right after the VAFU Context
	pSource = pWSUsrVirt + sizeof(VAFU2_CNTXT);
	
	// The destination buffer is right after the source buffer
	pDest   = pSource + a_num_bytes;
	
	pSourceCL = reinterpret_cast<struct OneCL *>(pSource);
	pDestCL   = reinterpret_cast<struct OneCL *>(pDest);
	
	// Note: the usage of the VAFU2_CNTXT structure here is specific to the underlying bitstream
	// implementation. The bitstream targeted for use with this sample application must implement
	// the Validation AFU 2 interface and abide by the contract that a VAFU2_CNTXT structure will
	// appear at byte offset 0 within the supplied AFU Context workspace.
	
	// Initialize the command buffer
	::memset(pVAFU2_cntxt, 0, sizeof(VAFU2_CNTXT));
	pVAFU2_cntxt->num_cl  = a_num_cl;
	pVAFU2_cntxt->pSource = pSource;
	pVAFU2_cntxt->pDest   = pDest;
   }
    
}

void SphereCollision::startTransaction(uint32_t value_times){
 if(0 == m_Result){
     pSource = pWSUsrVirt + sizeof(VAFU2_CNTXT);
	timespec ts;
	timespec tf;




	INFO("Starting SPL Transaction with Workspace");
	m_SPLService->StartTransactionContext(TransactionID(), pWSUsrVirt, 100);
	m_Sem.Wait();
	
	
	
	
	bt32bitInt count(500);  // 5 seconds with 10 millisecond sleep
      bt32bitInt delay(1000);   // 10 milliseconds is the default
	  
	  	float num1 = 6.5;
        float num2 = 7.5;
        
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        cout << "SIZEEEEE" << colInfosList.size() << "\n";
        
        float *inputs_AFU = (float*)malloc(sizeof(float)*((colInfosList.size()*8)+10));
	volatile float *boardIn = (float*)pSource;
		
        int indexinput = 16;
        collisionInfo auxinfo;
        std::list<collisionInfo>::iterator it;
		for (it = colInfosList.begin(); it != colInfosList.end(); it++) {
			auxinfo = *it;
                        inputs_AFU[indexinput] = auxinfo.x1;
                        inputs_AFU[indexinput+1] = auxinfo.x2;
                        inputs_AFU[indexinput+2] = auxinfo.x3;
                        inputs_AFU[indexinput+3] = auxinfo.r1;
                        inputs_AFU[indexinput+4] = auxinfo.y1;
                        inputs_AFU[indexinput+5] = auxinfo.y2;
                        inputs_AFU[indexinput+6] = auxinfo.y3;
                        inputs_AFU[indexinput+7] = auxinfo.r2;
                        indexinput = indexinput + 8;
		}
        
        
        
        
        uint32_t value_in = value_times;
        inputs_AFU[2] = reinterpret_cast<float &>(value_in);
		
	memcpy((void*)boardIn, inputs_AFU, sizeof(float)*((colInfosList.size()*8)+12));	
	
        
        //inputs_AFU[9] = 2.0000002; //1
        //inputs_AFU[9] = 2.0000024; //10
		//inputs_AFU[9] = 2.2384186; //1 million
        /*for(int i = 0; i < 16*9; i++){
             inputs_AFU[i] = num1;
            
        }*/
        
        
        
        
	
	    btUnsignedInt        cl;               // Loop counter. Cache-Line number.
	int                  tres;              // If many errors in buffer, only dump a limited number
	btInt                res = 0;
	ostringstream        oss("");          // Place to stash fancy strings
	btUnsigned32bitInt   tCacheLine[16];   // Temporary cacheline for various purposes
	CASSERT( sizeof(tCacheLine) == CL(1) );
	
		int modules_num = value_in*8*8;
        
        float *pu32 = reinterpret_cast<float*>(&pDestCL[0]); 
        //pu32 = pu32+15*7;
        
//#####################################################################################################
//#####################################################################################################
//######################################## START 1ST COLLISION ########################################
//#####################################################################################################
//#####################################################################################################

	inputs_AFU[0] = 3.700012;

	memcpy((void*)boardIn, inputs_AFU, sizeof(float));
     
      volatile bt32bitInt done = pVAFU2_cntxt->Status & VAFU2_CNTXT_STATUS_DONE;
      int counter = 0;
      while (!done && --count) {
         //SleepMilli( delay );
		 SleepMilli(1000);
         done = pVAFU2_cntxt->Status & VAFU2_CNTXT_STATUS_DONE;
         counter = counter +1;
         if(counter == 60){
            break;
             
        }
      }
        
        
        
        
		
		
//#####################################################################################################
//#####################################################################################################		
//######################################## END 1ST COLLISION ##########################################
//#####################################################################################################
//#####################################################################################################		
int countdataout = 0;
        for(int i = 0;i< modules_num;i++){
            if(i%8 == 0){ cout << "address " << countdataout << "\n";countdataout++;}
            cout << *pu32 << "\n";
            ++pu32;
        }
//cout << *pu32 << "\n";


	
		


	
    //Issue Stop Transaction and wait for OnTransactionStopped
	INFO("Stopping SPL Transaction");
	m_SPLService->StopTransactionContext(TransactionID());
        m_Sem.Wait();
        
	
   }

   ////////////////////////////////////////////////////////////////////////////
   // Clean up and exit
   INFO("Workspace verification complete, freeing workspace.");
   m_SPLService->WorkspaceFree(m_pWkspcVirt, TransactionID());
   m_Sem.Wait();

   //m_runtimClient->end();
 // while(1){}
}


btInt SphereCollision::run(uint32_t value_times)
{
   cout <<"======================="<<endl;
   cout <<"= Hello SPL LB Sample ="<<endl;
   cout <<"======================="<<endl;

   // Request our AFU.

   // NOTE: This example is bypassing the Resource Manager's configuration record lookup
   //  mechanism.  This code is work around code and subject to change.
   NamedValueSet Manifest;
   NamedValueSet ConfigRecord;


#if defined( HWAFU )                /* Use FPGA hardware */
   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_SERVICE_NAME, "libHWSPLAFU");
   ConfigRecord.Add(keyRegAFU_ID,"5DA62813-9A75-4228-8FDB-5D4006DD55CE");
   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_AIA_NAME, "libAASUAIA");

   #elif defined ( ASEAFU )
   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_SERVICE_NAME, "libASESPLAFU");
   ConfigRecord.Add(AAL_FACTORY_CREATE_SOFTWARE_SERVICE,true);

#else

   ConfigRecord.Add(AAL_FACTORY_CREATE_CONFIGRECORD_FULL_SERVICE_NAME, "libSWSimSPLAFU");
   ConfigRecord.Add(AAL_FACTORY_CREATE_SOFTWARE_SERVICE,true);
#endif

   Manifest.Add(AAL_FACTORY_CREATE_CONFIGRECORD_INCLUDED, ConfigRecord);

   Manifest.Add(AAL_FACTORY_CREATE_SERVICENAME, "Hello SPL LB");

   MSG("Allocating Service");

   // Allocate the Service and allocate the required workspace.
   //   This happens in the background via callbacks (simple state machine).
   //   When everything is set we do the real work here in the main thread.
   m_runtimClient->getRuntime()->allocService(dynamic_cast<IBase *>(this), Manifest);

   m_Sem.Wait();

   // If all went well run test.
   //   NOTE: If not successful we simply bail.
   //         A better design would do all appropriate clean-up.
   if(0 == m_Result){


	//=============================
	// Now we have the NLB Service
	//   now we can use it
	//=============================
	MSG("Running Test");
	
	pWSUsrVirt = m_pWkspcVirt; // Address of Workspace
	WSLen      = m_WkspcSize; // Length of workspace
	
	INFO("Allocated " << WSLen << "-byte Workspace at virtual address "
					<< std::hex << (void *)pWSUsrVirt);
	
	// Number of bytes in each of the source and destination buffers (4 MiB in this case)
	a_num_bytes= (btUnsigned32bitInt) ((WSLen - sizeof(VAFU2_CNTXT)) / 2);
	a_num_cl   = a_num_bytes / CL(1);  // number of cache lines in buffer
	
	// VAFU Context is at the beginning of the buffer
	pVAFU2_cntxt = reinterpret_cast<VAFU2_CNTXT *>(pWSUsrVirt);
	
	// The source buffer is right after the VAFU Context
	pSource = pWSUsrVirt + sizeof(VAFU2_CNTXT);
	
	// The destination buffer is right after the source buffer
	pDest   = pSource + a_num_bytes;
	
	pSourceCL = reinterpret_cast<struct OneCL *>(pSource);
	pDestCL   = reinterpret_cast<struct OneCL *>(pDest);
	
	// Note: the usage of the VAFU2_CNTXT structure here is specific to the underlying bitstream
	// implementation. The bitstream targeted for use with this sample application must implement
	// the Validation AFU 2 interface and abide by the contract that a VAFU2_CNTXT structure will
	// appear at byte offset 0 within the supplied AFU Context workspace.
	
	// Initialize the command buffer
	::memset(pVAFU2_cntxt, 0, sizeof(VAFU2_CNTXT));
	pVAFU2_cntxt->num_cl  = a_num_cl;
	pVAFU2_cntxt->pSource = pSource;
	pVAFU2_cntxt->pDest   = pDest;
	
	INFO("VAFU2 Context=" << std::hex << (void *)pVAFU2_cntxt <<
		" Src="          << std::hex << (void *)pVAFU2_cntxt->pSource <<
		" Dest="         << std::hex << (void *)pVAFU2_cntxt->pDest << std::dec);
	timespec ts;
	timespec tf;




	INFO("Starting SPL Transaction with Workspace");
	m_SPLService->StartTransactionContext(TransactionID(), pWSUsrVirt, 100);
	m_Sem.Wait();
	
	
	
	
	bt32bitInt count(500);  // 5 seconds with 10 millisecond sleep
      bt32bitInt delay(1000);   // 10 milliseconds is the default
	  
	  	float num1 = 6.5;
        float num2 = 7.5;
        
	     
        
        
	float *inputvalue_timess_AFU = (float*)malloc(sizeof(float)*(16*27));
        float *inputs_AFU = (float*)malloc(sizeof(float)*(16*27));
	volatile float *boardIn = (float*)pSource;
		
		
		//inputs_AFU[1] =4.0;
		/*float num = 0.1;
		for (int i = 32; i<=16*27; i=i+8){
			inputs_AFU[i] =  num;
			inputs_AFU[i+1] =num;
			inputs_AFU[i+2] =num;
			inputs_AFU[i+3] =num;
			inputs_AFU[i+4] =num;
			inputs_AFU[i+5] =num;
			inputs_AFU[i+6] =num;
			inputs_AFU[i+7] =num;
                        num = num + 0.1;
			
		}    */
        /*int *times_AFU = (int*)malloc(sizeof(int));
	volatile int *timeIn = (int*)(pSource); 
        timeIn = timeIn + 9;
        
	
        times_AFU[0] = 2;
        memcpy((void*)timeIn, inputs_AFU, sizeof(int));*/
        uint32_t value_in = value_times;
        inputs_AFU[2] = reinterpret_cast<float &>(value_in);
		
	memcpy((void*)boardIn, inputs_AFU, sizeof(float)*16*29);	
	
        
        //inputs_AFU[9] = 2.0000002; //1
        //inputs_AFU[9] = 2.0000024; //10
		//inputs_AFU[9] = 2.2384186; //1 million
        /*for(int i = 0; i < 16*9; i++){
             inputs_AFU[i] = num1;
            
        }*/
        
        
        
        
	
	    btUnsignedInt        cl;               // Loop counter. Cache-Line number.
	int                  tres;              // If many errors in buffer, only dump a limited number
	btInt                res = 0;
	ostringstream        oss("");          // Place to stash fancy strings
	btUnsigned32bitInt   tCacheLine[16];   // Temporary cacheline for various purposes
	CASSERT( sizeof(tCacheLine) == CL(1) );
	
		int modules_num = value_in*8*8;
        
        float *pu32 = reinterpret_cast<float*>(&pDestCL[0]); 
        //pu32 = pu32+15*7;
        
//#####################################################################################################
//#####################################################################################################
//######################################## START 1ST COLLISION ########################################
//#####################################################################################################
//#####################################################################################################

	inputs_AFU[0] = 3.700012;

	memcpy((void*)boardIn, inputs_AFU, sizeof(float));
     
      volatile bt32bitInt done = pVAFU2_cntxt->Status & VAFU2_CNTXT_STATUS_DONE;
      int counter = 0;
      while (!done && --count) {
         //SleepMilli( delay );
		 SleepMilli(1000);
         done = pVAFU2_cntxt->Status & VAFU2_CNTXT_STATUS_DONE;
         counter = counter +1;
         if(counter == 60){
            break;
             
        }
      }
        
        
        
        
		
		
//#####################################################################################################
//#####################################################################################################		
//######################################## END 1ST COLLISION ##########################################
//#####################################################################################################
//#####################################################################################################		
int countdataout = 0;
        for(int i = 0;i< modules_num;i++){
            if(i%8 == 0){ cout << "address " << countdataout << "\n";countdataout++;}
            cout << *pu32 << "\n";
            ++pu32;
        }
//cout << *pu32 << "\n";


	
		


	
    //Issue Stop Transaction and wait for OnTransactionStopped
	INFO("Stopping SPL Transaction");
	m_SPLService->StopTransactionContext(TransactionID());
        m_Sem.Wait();
        
	
   }

   ////////////////////////////////////////////////////////////////////////////
   // Clean up and exit
   INFO("Workspace verification complete, freeing workspace.");
   m_SPLService->WorkspaceFree(m_pWkspcVirt, TransactionID());
   m_Sem.Wait();

   m_runtimClient->end();
 // while(1){}
   return m_Result;
}



























//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################
//########################################################################### ODE FUNCTIONS ##################################################################################################





static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	dContact contact;
	contact.surface.mode = dContactBounce | dContactSoftCFM;
	// friction parameter
	contact.surface.mu = dInfinity;
	// bounce is the amount of "bouncyness".
	contact.surface.bounce = 1.0;
	// bounce_vel is the minimum incoming velocity to cause a bounce
	contact.surface.bounce_vel = 0.1; //0.1
									  // constraint force mixing parameter
	contact.surface.soft_cfm = 0.001;
	if (int numc = dCollide(o1, o2, 1, &contact.geom, sizeof(dContact))) {
		dJointID c = dJointCreateContact(world, contactgroup, &contact);
		dJointAttach(c, b1, b2);
	}
}

// start simulation - set viewpoint (camera)
static void start()
{
	// Original View
	static float xyz[3] = { 3.0f, -3.0f, 8.0f };
	static float hpr[3] = { 136.000f,-50.0000f,0.0000f };
	// Top Down View
	//    static float xyz[3] = {0.0f,-8.0f,1.0f};
	//   static float hpr[3] = {90.0f,0.0f,0.0f};
	//dBodyAddForce(sphere0, 100, -100, 0);
	dsSetViewpoint(xyz, hpr);
}
static SphereCollision *theApp;

// simulation loop
static void simLoop(int pause)
{
	const dReal *pos;
	const dReal *R;





	// force for the spheres
	if (pause == 1) {
		// find collisions and add contact joints
		dSpaceCollide(space, 0, &nearCallback);
                
                //theApp->startTransaction(6);
                
		// step the simulation
		dWorldQuickStep(world, 0.01);
		// remove all contact joints
		dJointGroupEmpty(contactgroup);
		// redraw sphere at new location
#ifdef GRAPH
		std::list<dGeomID>::iterator it;
		for (it = spheresList.begin(); it != spheresList.end(); it++) {
			pos = dGeomGetPosition((*it));
			R = dGeomGetRotation((*it));
			dsDrawSphere(pos, R, dGeomSphereGetRadius((*it)));
		}
#endif
	}
	else {
#ifdef GRAPH
		std::list<dGeomID>::iterator it;
		for (it = spheresList.begin(); it != spheresList.end(); it++) {
			pos = dGeomGetPosition((*it));
			R = dGeomGetRotation((*it));
			dsDrawSphere(pos, R, dGeomSphereGetRadius((*it)));
		}
#endif
	}
}



static void createDiamond(float rinn, float rn, float zin, float increase_distance, int topBottom, float zcentral /*top = 1, botton=0*/) {
	float rin = rinn;
	float r = rn;
	float hip = sqrt(pow(2 * r, 2.0) + pow(2 * r, 2.0));
	float dbtwnball = (hip - 2 * r)+ increase_distance;
	float rmax = rin;
	float x = 0;
	float y = 0;
	float z = zin;
	int incr = 1;
	int incr2 = 1;

	x = 0.5*dbtwnball + ((rin / 2) - 1)*dbtwnball + ((rin / 2) - 0.5)*(2 * r);
	float zforce = 4;
	//x = (rin - (0 + 1)*0.5)*dbtwnball + (rin - (0.5*(0 + 1))) * 2 * r;
	for (int j = 0; j < rmax; j++) {
		if (j == 0) {
			dBodyID sphere0;
			dGeomID sphere0_geom;
			sphere0 = dBodyCreate(world);
			sphere0_geom = dCreateSphere(space, r);
			dMassSetSphere(&m, 1, r);
			dBodySetMass(sphere0, &m);
			dGeomSetBody(sphere0_geom, sphere0);
			dBodySetPosition(sphere0, x, y, z);
			//dBodyAddForce(sphere0, 0, 0, zforce);
			spheresList.push_back(sphere0_geom);

		}
		else {
			x = x - dbtwnball / 2;
			x = x - r;

			if (j == 1) {
				y = (dbtwnball / 2) + r;
				dBodyID sphere0;
				dGeomID sphere0_geom;
				sphere0 = dBodyCreate(world);
				sphere0_geom = dCreateSphere(space, r);
				dMassSetSphere(&m, 0.5, r);
				dBodySetMass(sphere0, &m);
				dGeomSetBody(sphere0_geom, sphere0);
				dBodySetPosition(sphere0, x, y, z);
				//dBodyAddForce(sphere0, 0, 0, zforce);
				spheresList.push_back(sphere0_geom);
				

				y = -y;
				sphere0 = dBodyCreate(world);
				sphere0_geom = dCreateSphere(space, r);
				dMassSetSphere(&m, 1, r);
				dBodySetMass(sphere0, &m);
				dGeomSetBody(sphere0_geom, sphere0);
				dBodySetPosition(sphere0, x, y, z);
				//dBodyAddForce(sphere0, 0, 0, zforce);
				spheresList.push_back(sphere0_geom);
			}
			else {
				if (j == 2) {
					dBodyID sphere0;
					dGeomID sphere0_geom;
					sphere0 = dBodyCreate(world);
					sphere0_geom = dCreateSphere(space, r);
					dMassSetSphere(&m, 0.5, r);
					dBodySetMass(sphere0, &m);
					dGeomSetBody(sphere0_geom, sphere0);
					dBodySetPosition(sphere0, x, 0, z);
					//dBodyAddForce(sphere0, 0, 0, zforce);
					spheresList.push_back(sphere0_geom);

					//y = (dbtwnball / 2) + 2 * r;
					y = dbtwnball + 2 * r;

					sphere0 = dBodyCreate(world);
					sphere0_geom = dCreateSphere(space, r);
					dMassSetSphere(&m, 1, r);
					dBodySetMass(sphere0, &m);
					dGeomSetBody(sphere0_geom, sphere0);
					dBodySetPosition(sphere0, x, y, z);
					//dBodyAddForce(sphere0, 0, 0, zforce);
					spheresList.push_back(sphere0_geom);

					y = -y;

					sphere0 = dBodyCreate(world);
					sphere0_geom = dCreateSphere(space, r);
					dMassSetSphere(&m, 1, r);
					dBodySetMass(sphere0, &m);
					dGeomSetBody(sphere0_geom, sphere0);
					dBodySetPosition(sphere0, x, y, z);
					//dBodyAddForce(sphere0, 0, 0, zforce);
					spheresList.push_back(sphere0_geom);
				}
				else {
					if (j % 2 == 1) {
						for (int k = 0; k < j - incr; k++) {
							y = (dbtwnball / 2) + (k*dbtwnball) + ((k*r * 2) + r);

							dBodyID sphere0;
							dGeomID sphere0_geom;
							sphere0 = dBodyCreate(world);
							sphere0_geom = dCreateSphere(space, r);
							dMassSetSphere(&m, 0.5, r);
							dBodySetMass(sphere0, &m);
							dGeomSetBody(sphere0_geom, sphere0);
							dBodySetPosition(sphere0, x, y, z);
							//dBodyAddForce(sphere0, 0, 0, zforce);
							spheresList.push_back(sphere0_geom);

							y = -y;

							sphere0 = dBodyCreate(world);
							sphere0_geom = dCreateSphere(space, r);
							dMassSetSphere(&m, 0.5, r);
							dBodySetMass(sphere0, &m);
							dGeomSetBody(sphere0_geom, sphere0);
							dBodySetPosition(sphere0, x, y, z);
							//dBodyAddForce(sphere0, 0, 0, zforce);
							spheresList.push_back(sphere0_geom);
						}
						incr++;

					}
					if (j % 2 == 0) {
						dBodyID sphere0;
						dGeomID sphere0_geom;
						sphere0 = dBodyCreate(world);
						sphere0_geom = dCreateSphere(space, r);
						dMassSetSphere(&m, 0.5, r);
						dBodySetMass(sphere0, &m);
						dGeomSetBody(sphere0_geom, sphere0);
						dBodySetPosition(sphere0, x, 0, z);
						//dBodyAddForce(sphere0, 0, 0, zforce);
						spheresList.push_back(sphere0_geom);
						for (int k = 1; k < j - incr2; k++) {
							y = (k*dbtwnball) + ((k*r * 2));



							sphere0 = dBodyCreate(world);
							sphere0_geom = dCreateSphere(space, r);
							dMassSetSphere(&m, 0.5, r);
							dBodySetMass(sphere0, &m);
							dGeomSetBody(sphere0_geom, sphere0);
							dBodySetPosition(sphere0, x, y, z);
							//dBodyAddForce(sphere0, 0, 0, zforce);
							spheresList.push_back(sphere0_geom);

							y = -y;

							sphere0 = dBodyCreate(world);
							sphere0_geom = dCreateSphere(space, r);
							dMassSetSphere(&m, 0.5, r);
							dBodySetMass(sphere0, &m);
							dGeomSetBody(sphere0_geom, sphere0);
							dBodySetPosition(sphere0, x, y, z);
							//dBodyAddForce(sphere0, 0, 0, zforce);
							spheresList.push_back(sphere0_geom);
						}
						incr2++;

					}
				}

			}


		}
	}

	x = 0;
	y = 0;
	z = zin;
	incr = 1;
	incr2 = 1;

	x = (0.5*dbtwnball + ((rin / 2) - 1)*dbtwnball + ((rin / 2) - 0.5)*(2 * r))*(-1);
	for (int j = 0; j < rmax - 1; j++) {
		if (j == 0) {
			dBodyID sphere0;
			dGeomID sphere0_geom;
			sphere0 = dBodyCreate(world);
			sphere0_geom = dCreateSphere(space, r);
			dMassSetSphere(&m, 1, r);
			dBodySetMass(sphere0, &m);
			dGeomSetBody(sphere0_geom, sphere0);
			dBodySetPosition(sphere0, x, y, z);
			//dBodyAddForce(sphere0, 0, 0, zforce);
			spheresList.push_back(sphere0_geom);
		}
		else {
			x = x + dbtwnball / 2;
			x = x + r;

			if (j == 1) {
				y = (dbtwnball / 2) + r;
				dBodyID sphere0;
				dGeomID sphere0_geom;
				sphere0 = dBodyCreate(world);
				sphere0_geom = dCreateSphere(space, r);
				dMassSetSphere(&m, 0.5, r);
				dBodySetMass(sphere0, &m);
				dGeomSetBody(sphere0_geom, sphere0);
				dBodySetPosition(sphere0, x, y, z);
				//dBodyAddForce(sphere0, 0, 0, zforce);
				spheresList.push_back(sphere0_geom);

				y = -y;
				sphere0 = dBodyCreate(world);
				sphere0_geom = dCreateSphere(space, r);
				dMassSetSphere(&m, 1, r);
				dBodySetMass(sphere0, &m);
				dGeomSetBody(sphere0_geom, sphere0);
				dBodySetPosition(sphere0, x, y, z);
				//dBodyAddForce(sphere0, 0, 0, zforce);
				spheresList.push_back(sphere0_geom);
			}
			else {
				if (j == 2) {
					dBodyID sphere0;
					dGeomID sphere0_geom;
					sphere0 = dBodyCreate(world);
					sphere0_geom = dCreateSphere(space, r);
					dMassSetSphere(&m, 0.5, r);
					dBodySetMass(sphere0, &m);
					dGeomSetBody(sphere0_geom, sphere0);
					dBodySetPosition(sphere0, x, 0, z);
					//dBodyAddForce(sphere0, 0, 0, zforce);
					spheresList.push_back(sphere0_geom);

					//y = (dbtwnball / 2) + 2 * r;
					y = dbtwnball + 2 * r;

					sphere0 = dBodyCreate(world);
					sphere0_geom = dCreateSphere(space, r);
					dMassSetSphere(&m, 1, r);
					dBodySetMass(sphere0, &m);
					dGeomSetBody(sphere0_geom, sphere0);
					dBodySetPosition(sphere0, x, y, z);
					//dBodyAddForce(sphere0, 0, 0, zforce);
					spheresList.push_back(sphere0_geom);

					y = -y;

					sphere0 = dBodyCreate(world);
					sphere0_geom = dCreateSphere(space, r);
					dMassSetSphere(&m, 1, r);
					dBodySetMass(sphere0, &m);
					dGeomSetBody(sphere0_geom, sphere0);
					dBodySetPosition(sphere0, x, y, z);
					//dBodyAddForce(sphere0, 0, 0, zforce);
					spheresList.push_back(sphere0_geom);
				}
				else {
					if (j % 2 == 1) {
						for (int k = 0; k < j - incr; k++) {
							y = (dbtwnball / 2) + (k*dbtwnball) + ((k*r * 2) + r);

							dBodyID sphere0;
							dGeomID sphere0_geom;
							sphere0 = dBodyCreate(world);
							sphere0_geom = dCreateSphere(space, r);
							dMassSetSphere(&m, 0.5, r);
							dBodySetMass(sphere0, &m);
							dGeomSetBody(sphere0_geom, sphere0);
							dBodySetPosition(sphere0, x, y, z);
							//dBodyAddForce(sphere0, 0, 0, zforce);
							spheresList.push_back(sphere0_geom);

							y = -y;

							sphere0 = dBodyCreate(world);
							sphere0_geom = dCreateSphere(space, r);
							dMassSetSphere(&m, 0.5, r);
							dBodySetMass(sphere0, &m);
							dGeomSetBody(sphere0_geom, sphere0);
							dBodySetPosition(sphere0, x, y, z);
							//dBodyAddForce(sphere0, 0, 0, zforce);
							spheresList.push_back(sphere0_geom);
						}
						incr++;

					}
					if (j % 2 == 0) {
						dBodyID sphere0;
						dGeomID sphere0_geom;
						sphere0 = dBodyCreate(world);
						sphere0_geom = dCreateSphere(space, r);
						dMassSetSphere(&m, 0.5, r);
						dBodySetMass(sphere0, &m);
						dGeomSetBody(sphere0_geom, sphere0);
						dBodySetPosition(sphere0, x, 0, z);
						//dBodyAddForce(sphere0, 0, 0, zforce);
						spheresList.push_back(sphere0_geom);
						for (int k = 1; k < j - incr2; k++) {
							y = (k*dbtwnball) + ((k*r * 2));



							sphere0 = dBodyCreate(world);
							sphere0_geom = dCreateSphere(space, r);
							dMassSetSphere(&m, 0.5, r);
							dBodySetMass(sphere0, &m);
							dGeomSetBody(sphere0_geom, sphere0);
							dBodySetPosition(sphere0, x, y, z);
							//dBodyAddForce(sphere0, 0, 0, zforce);
							spheresList.push_back(sphere0_geom);

							y = -y;

							sphere0 = dBodyCreate(world);
							sphere0_geom = dCreateSphere(space, r);
							dMassSetSphere(&m, 0.5, r);
							dBodySetMass(sphere0, &m);
							dGeomSetBody(sphere0_geom, sphere0);
							dBodySetPosition(sphere0, x, y, z);
							//dBodyAddForce(sphere0, 0, 0, zforce);
							spheresList.push_back(sphere0_geom);
						}
						incr2++;

					}
				}

			}


		}
	}
}

static void createPrism(float rMaxBalls, float rball, float increase_distance,float zcentral) {
	float rin = rMaxBalls;
	float r = rball;
	float hip = sqrt(pow(2 * r, 2.0) + pow(2 * r, 2.0));
	float dbtwnball = (hip - 2 * r)+ increase_distance;

	for (int i = 1; i <= rin; i++) {
		createDiamond(i, r, ((i - 1)*dbtwnball)+(r*2)+4, increase_distance,0, zcentral);

	}
	int increase = rin;
	for (int i = rin - 1; i >= 1; i--) {
		createDiamond(i, r, (increase*dbtwnball)+(r*2)+4, increase_distance,0, zcentral);
		increase++;

	}
}


static int countTime = 0;
static int countColisions = 0;
static int num_collisions = 1;
static int num_float_ops = 0;


static int dCollideSpheres3 (dVector3 p1, dReal r1,

                     dVector3 p2, dReal r2, dContactGeom *c)

{
/*float *inputs_AFU = (float*)malloc(sizeof(float)*8);
volatile float *boardIn = (float*)pSource;
inputs_AFU = (float*)malloc(sizeof(float)*8);
boardIn = (float*)pSource;
inputs_AFU[0] = p1[0];
inputs_AFU[1] = p1[1];
inputs_AFU[2] = p1[2];
inputs_AFU[3] = r1;
inputs_AFU[4] = p2[0];
inputs_AFU[5] = p2[1];
inputs_AFU[6] = p2[2];
inputs_AFU[7] = r2;
memcpy((void*)boardIn, inputs_AFU, sizeof(float)*8);	
pSource = pSource + 8;
cout << "COPIED \n";

collisionInfo collisonin;
  collisonin.x1 = p1[0];  
  collisonin.x2 = p1[1];
  collisonin.x3 = p1[2];
  collisonin.r1 = r1;
  collisonin.y1 = p2[0];  
  collisonin.y2 = p2[1];
  collisonin.y3 = p2[2];
  collisonin.r2 = r2;
  
  colInfosList.push_back(collisonin);*/
    

    dReal d = dCalcPointsDistance3(p1,p2);
if (d > (r1 + r2)){
	return 0;
	
} 

    if (d <= 0) {

        c->pos[0] = p1[0];

        c->pos[1] = p1[1];

        c->pos[2] = p1[2];

        c->normal[0] = 1;

        c->normal[1] = 0;

        c->normal[2] = 0;

        c->depth = r1 + r2;
		//num_float_ops++;
		return 1;
    

    }

    else {

    

        dReal d1 = dRecip (d);

        c->normal[0] = (p1[0]-p2[0])*d1;

        c->normal[1] = (p1[1]-p2[1])*d1;

        c->normal[2] = (p1[2]-p2[2])*d1;

        dReal k = REAL(0.5) * (r2 - r1 - d);

        c->pos[0] = p1[0] + c->normal[0]*k;

        c->pos[1] = p1[1] + c->normal[1]*k;

        c->pos[2] = p1[2] + c->normal[2]*k;

        c->depth = r1 + r2 - d;
		
		return 2;
        

    }
    

}









int SphereCollision::dCollideSphereSphereAAL (dxGeom *o1, dxGeom *o2, int flags,

                          dContactGeom *contact, int skip)

{

 

     dIASSERT (skip >= (int)sizeof(dContactGeom));

    dIASSERT (o1->type == dSphereClass);

    dIASSERT (o2->type == dSphereClass);

    dIASSERT ((flags & NUMC_MASK) >= 1);



    dxSphere *sphere1 = (dxSphere*) o1;

    dxSphere *sphere2 = (dxSphere*) o2;



    contact->g1 = o1;

    contact->g2 = o2;

    contact->side1 = -1;

    contact->side2 = -1;
	int test;
	long int durationtotal = 0;
	for(int i=0;i<10;i++){
	
		high_resolution_clock::time_point t1 = high_resolution_clock::now();
	
		test = dCollideSpheres3 (o1->final_posr->pos,sphere1->radius,o2->final_posr->pos,sphere2->radius,contact);
		high_resolution_clock::time_point t2 = high_resolution_clock::now();
		
		auto duration = duration_cast<nanoseconds>( t2 - t1 ).count();
		durationtotal = durationtotal + duration;
		
	}
	
	durationtotal = durationtotal/10;
	if(countColisions >= num_collisions){		
		if(num_collisions < 101000){
			cout << "AVERAGE " << num_collisions << "\t" << countTime  << "\t" << countTime/num_collisions << "\t" << num_float_ops << "\n";
			num_collisions = num_collisions + 1;
			//countColisions = 0;
			//countTime = 0;
		}
	}else {
	if(test == 2){
		countColisions = countColisions +1;
		countTime = countTime + durationtotal;
		num_float_ops = num_float_ops + 18;
	}
		
	}
    
	if(test  > 0){
		test = 1;
	}
	

    return test;

}



int main (int argc, char **argv)

{
       /* btInt Result;
        RuntimeClient  runtimeClient;
        //SphereCollision theApp(&runtimeClient, argv[1]);
        theApp = new SphereCollision(&runtimeClient, argv[1]);
        
        if(!runtimeClient.isOK()){
            ERR("Runtime Failed to Start");
            exit(1);
        }
        
        theApp->initialize();*/
    
   // setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.stop = 0;
	fn.command = 0;
	fn.path_to_textures = "../../drawstuff/textures";
	printf("aaaaa");
	dInitODE();
	dSetColliderOverride(0, 0, SphereCollision::dCollideSphereSphereAAL);
	// create world
	world = dWorldCreate();
	space = dHashSpaceCreate(0);
	dWorldSetGravity(world, 0, 0, 0); //Original Gravity = -0.2 
	dWorldSetCFM(world, 1e-5);
	dCreatePlane(space, 0, 0, 1, 0);
	contactgroup = dJointGroupCreate(0);


	float rmax = 50;
	float rball = 0.1;
	float increase_distance = 0;
	float hip = sqrt(pow(2 * rball, 2.0) + pow(2 * rball, 2.0));
	float dbtwnball = (hip - 2 * rball) + increase_distance;
	float zcentral = (0.5*dbtwnball + ((rmax / 2) - 1)*dbtwnball + (rmax / 2)*(2 * rball));
	createPrism(rmax, rball, increase_distance, zcentral);





	// run simulation
	dsSimulationLoop(argc, argv, 352, 288, &fn);
	// clean up
	dJointGroupDestroy(contactgroup);
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();
	return 0;

}