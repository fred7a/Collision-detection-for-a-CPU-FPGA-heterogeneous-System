// Copyright (c) 2007-2015, Intel Corporation
//
// Redistribution  and  use  in source  and  binary  forms,  with  or  without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of  source code  must retain the  above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name  of Intel Corporation  nor the names of its contributors
//   may be used to  endorse or promote  products derived  from this  software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,  BUT NOT LIMITED TO,  THE
// IMPLIED WARRANTIES OF  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT  SHALL THE COPYRIGHT OWNER  OR CONTRIBUTORS BE
// LIABLE  FOR  ANY  DIRECT,  INDIRECT,  INCIDENTAL,  SPECIAL,  EXEMPLARY,  OR
// CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT LIMITED  TO,  PROCUREMENT  OF
// SUBSTITUTE GOODS OR SERVICES;  LOSS OF USE,  DATA, OR PROFITS;  OR BUSINESS
// INTERRUPTION)  HOWEVER CAUSED  AND ON ANY THEORY  OF LIABILITY,  WHETHER IN
// CONTRACT,  STRICT LIABILITY,  OR TORT  (INCLUDING NEGLIGENCE  OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,  EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************
/// @file dCollideSpheres.cpp
/// @brief SPL enabled version of dCollideSpheres.
/// @ingroup dCollideSpheres
/// @verbatim
/// Intel(R) QuickAssist Technology Accelerator Abstraction Layer Sample Application
///
///    This application is for example purposes only.
///    It is not intended to represent a model for developing commercially-deployable applications.
///    It is designed to show working examples of the AAL programming model and APIs.
///
/// AUTHORS: Fredy Alves, Federeal University of Viçosa. David Sheffield, Intel Corporation.
///
/// This Sample demonstrates the following:
///    - Doing dCollideSpheres within the SPL framework
///
/// This sample is designed to be used with the SPLAFU Service.
//****************************************************************************
//#include <ode/ode.h>
//#include </home/fredy/Documents/ode-0.13/include/ode/collision.h>

//#include </home/fredy/Documents/ode-0.13/tests/UnitTest++/src/UnitTest++.h>
#include <aalsdk/AAL.h>
#include <aalsdk/xlRuntime.h>
#include <aalsdk/AALLoggerExtern.h> // Logger


#include <aalsdk/service/ISPLAFU.h>       // Service Interface
#include <aalsdk/service/ISPLClient.h>    // Service Client Interface
#include <aalsdk/kernel/vafu2defs.h>      // AFU structure definitions (brings in spl2defs.h)

#include <string.h>
#include <cassert>
#include <sys/time.h>
//#include <chrono>
#include <iostream>
#include <fstream>

//****************************************************************************
// UN-COMMENT appropriate #define in order to enable either Hardware or ASE.
//    DEFAULT is to use Software Simulation.
//****************************************************************************
//#define  HWAFU
#define  ASEAFU

using namespace AAL;
using namespace std;
//using namespace std::chrono;
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

/// @addtogroup dCollideSpheresSample
/// @{


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
   MSG("Runtime stopped");
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
   MSG("Runtime Allocate Service Succeeded");
}

void RuntimeClient::runtimeEvent(const IEvent &rEvent)
{
   MSG("Generic message handler (runtime)");
}

IRuntime * RuntimeClient::getRuntime()
{
   return m_pRuntime;
}


/// @brief   Define our Service client class so that we can receive Service-related notifications from the AAL Runtime.
///          The Service Client contains the application logic.
///
/// When we request an AFU (Service) from AAL, the request will be fulfilled by calling into this interface.
class dCollideSpheres: public CAASBase, public IServiceClient, public ISPLClient
{
public:

   dCollideSpheres(RuntimeClient * rtc, char *puzName);
   ~dCollideSpheres();

   btInt run(uint32_t value_times, uint32_t addr1, uint32_t addr2);

   // <ISPLClient>
   virtual void OnTransactionStarted(TransactionID const &TranID,
                                     btVirtAddr AFUDSM,
                                     btWSSize AFUDSMSize);
   void _DumpCL(void *pCL,
                ostringstream &oss);
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

   /* SW implementation of a dCollideSpheres solver */
   static void print_board(uint32_t *board);
   static int32_t dCollideSpheres_norec(uint32_t *board, uint32_t *os);
   static int32_t check_correct(uint32_t *board, uint32_t *unsolved_pieces);
   static int32_t solve(uint32_t *board, uint32_t *os);
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


///////////////////////////////////////////////////////////////////////////////
///
///  Implementation
///
///////////////////////////////////////////////////////////////////////////////
dCollideSpheres::dCollideSpheres(RuntimeClient *rtc, char *puzName) :
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



dCollideSpheres::~dCollideSpheres()
{
   m_Sem.Destroy();
}

static long long int timescollideavg = 0;

int dCollideSpheres::run(uint32_t value_times, uint32_t addr1, uint32_t addr2)
{
   int runOk = 1;
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
	
	btVirtAddr         pWSUsrVirt = m_pWkspcVirt; // Address of Workspace
	const btWSSize     WSLen      = m_WkspcSize; // Length of workspace
	
	INFO("Allocated " << WSLen << "-byte Workspace at virtual address "
					<< std::hex << (void *)pWSUsrVirt);
	
	// Number of bytes in each of the source and destination buffers (4 MiB in this case)
	btUnsigned32bitInt a_num_bytes= (btUnsigned32bitInt) ((WSLen - sizeof(VAFU2_CNTXT)) / 2);
	btUnsigned32bitInt a_num_cl   = a_num_bytes / CL(1);  // number of cache lines in buffer
	
	// VAFU Context is at the beginning of the buffer
	VAFU2_CNTXT       *pVAFU2_cntxt = reinterpret_cast<VAFU2_CNTXT *>(pWSUsrVirt);
	
	// The source buffer is right after the VAFU Context
	btVirtAddr         pSource = pWSUsrVirt + sizeof(VAFU2_CNTXT);
	
	// The destination buffer is right after the source buffer
	btVirtAddr         pDest   = pSource + a_num_bytes;
	
	struct OneCL {                      // Make a cache-line sized structure
	btUnsigned32bitInt dw[16];       //    for array arithmetic
	};
	struct OneCL      *pSourceCL = reinterpret_cast<struct OneCL *>(pSource);
	struct OneCL      *pDestCL   = reinterpret_cast<struct OneCL *>(pDest);
	
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
	  
        float num1 = 1.0;
        
        float *inputs_AFU = (float*)malloc(sizeof(float)*(((value_times*2)+10)*16));
	volatile float *boardIn = (float*)pSource;
        int numAddressesLines = (value_times)*16;
        int numAddressesLines2 = (value_times*2)*16;
        /* uint32_t address1 = 655370;
		 uint32_t address2 = 720907;
		uint32_t address3 = 786444;
		uint32_t address4 = 851981;
		uint32_t address5 = 917518;
		uint32_t address6 = 983055;        
        uint32_t address7 = 1048592;
		uint32_t address8 = 1114129;   
        uint32_t address9 = 1179666;
		uint32_t address10 = 1245203;  
        uint32_t address11 = 1310740;
		uint32_t address12 = 1376277; 
        uint32_t address13 = 1441814;
		uint32_t address14 = 1507351; 
        uint32_t address15 = 1572888; 
        uint32_t address16 = 1638425;*/
		 
		/*uint32_t address1 = 720906;
        uint32_t address2 = 720906;
		uint32_t address3 = 720906;
		uint32_t address4 = 720906;
		uint32_t address5 = 720906;
		uint32_t address6 = 720906;      
        uint32_t address7 = 720906;
		uint32_t address8 = 720906;  
        uint32_t address9 = 720906;
		uint32_t address10 =720906;  
        uint32_t address11 =720906;
		uint32_t address12 =720906; 
        uint32_t address13 =720906;
		uint32_t address14 =720906; 
        uint32_t address15 =720906; 
        uint32_t address16 =720906;*/
		
         uint32_t address1 = 720906; //10-11
         uint32_t address2 = 851980; //12-13
		 uint32_t address3 = 983054; //14-15
		 uint32_t address4 = 1114128; //16-17
		 uint32_t address5 = 1245202; //18-19
		 uint32_t address6 = 1376276; //20-21        
         uint32_t address7 = 720906; //10-11
		 uint32_t address8 = 851980; //12-13  
         uint32_t address9 = 983054; //14-15
		 uint32_t address10= 1114128; //16-17  
         uint32_t address11= 1245202; //18-19
		 uint32_t address12= 1376276; //20-21 
         uint32_t address13= 720906; //10-11
		 uint32_t address14= 851980; //12-13  
         uint32_t address15= 983054; //14-15 
         uint32_t address16= 1114128; //16-17		
		
      /* uint32_t address1 = 2953510957;
        uint32_t address2 = 687908866;
	uint32_t address3 = 40897024;
	uint32_t address4 = 2424984;
	uint32_t address5 = 2281844745;
	uint32_t address6 = 134226176;      
        uint32_t address7 = 7864816;
	uint32_t address8 = 2953248797  ;
        uint32_t address9 = 419457025;
	uint32_t address10 =24118784;  
        uint32_t address11 =1376344;
	uint32_t address12 =1208037381; 
        uint32_t address13 =67113216;
	uint32_t address14 =3670256; 
        uint32_t address15 =2952986637 ;
        uint32_t address16 =10240;*/
        
        
        uint32_t value_intest = 20;
        float testvalue1 = reinterpret_cast<float &>(value_intest);		
		
        int i;
	for(i = 16; i <= numAddressesLines; i=i+8){
		//cout <<  "INDEX :" << i << "\n";
		if(num1 == (value_times*4)+1){
                    break;
                }else{
					//if(num1 == 19){
					/*		inputs_AFU[i] =  testvalue1;
                    inputs_AFU[i+1] =testvalue1;
                    inputs_AFU[i+2] =testvalue1;
                    inputs_AFU[i+3] =testvalue1;
                    num1 = num1 + 1.0;
                    inputs_AFU[i+4] =testvalue1;
                    inputs_AFU[i+5] =testvalue1;
                    inputs_AFU[i+6] =testvalue1;
                    inputs_AFU[i+7] =testvalue1;
                    num1 = num1 + 1.0;
					}else{*/
						
					/*inputs_AFU[i] =  num1;
                    inputs_AFU[i+1] =num1;
                    inputs_AFU[i+2] =num1;
                    inputs_AFU[i+3] =num1;
                    num1 = num1 + 1.0;
                    inputs_AFU[i+4] =num1;
                    inputs_AFU[i+5] =num1;
                    inputs_AFU[i+6] =num1;
                    inputs_AFU[i+7] =num1;
                    num1 = num1 + 1.0;*/
                    
					inputs_AFU[i] =  0.141421;
                    inputs_AFU[i+1] =0 ;
                    inputs_AFU[i+2] =4.28284 ;
                    inputs_AFU[i+3] =0.1;
                    num1 = num1 + 1.0;
                    inputs_AFU[i+4] =0 ;
                    inputs_AFU[i+5] =0 ;
                    inputs_AFU[i+6] =4.2 ;
                    inputs_AFU[i+7] =0.1;
                    num1 = num1 + 1.0;	
					
					//inputs_AFU[i] =  0.141421;
                    //inputs_AFU[i+1] =0 ;
                    //inputs_AFU[i+2] =4.28284 ;
                    //inputs_AFU[i+3] =0.1;
                    //num1 = num1 + 1.0;
					//inputs_AFU[i] =  0.141421;
                    //inputs_AFU[i+1] =0 ;
                    //inputs_AFU[i+2] =4.28284 ;
                    //inputs_AFU[i+3] =0.1;
                    //num1 = num1 + 1.0;					
					
					
					//}
                    
                    
                    
                    
                    
                }
                
	}	
	
	for(int j = i; j <= numAddressesLines2; j=j+16){
            
                    inputs_AFU[j] =  reinterpret_cast<float &>(address1);
		    inputs_AFU[j+1] =  reinterpret_cast<float &>(address2);
		    inputs_AFU[j+2] = reinterpret_cast<float &>(address3);
		    inputs_AFU[j+3] = reinterpret_cast<float &>(address4);
                    inputs_AFU[j+4] = reinterpret_cast<float &>(address5);
		    inputs_AFU[j+5] = reinterpret_cast<float &>(address6);
                    inputs_AFU[j+6] = reinterpret_cast<float &>(address7);
                    inputs_AFU[j+7] = reinterpret_cast<float &>(address8);
                    inputs_AFU[j+8] = reinterpret_cast<float &>(address9);
                    inputs_AFU[j+9] = reinterpret_cast<float &>(address10);
                    inputs_AFU[j+10] = reinterpret_cast<float &>(address11);
                    inputs_AFU[j+11] = reinterpret_cast<float &>(address12);
                    inputs_AFU[j+12] = reinterpret_cast<float &>(address13);
                    inputs_AFU[j+13] = reinterpret_cast<float &>(address14);
                    inputs_AFU[j+14] = reinterpret_cast<float &>(address15);
                    inputs_AFU[j+15] = reinterpret_cast<float &>(address16);
        }
        
        uint32_t value_in = value_times;
        //uint32_t value_in2 = 30;
        //uint32_t value_in3 = 50;
        uint32_t value_in2 = addr1 + value_times;
        uint32_t value_in3 = addr2;
        inputs_AFU[2] = reinterpret_cast<float &>(value_in);
        inputs_AFU[3] = reinterpret_cast<float &>(value_in2);
        inputs_AFU[4] = reinterpret_cast<float &>(value_in3);
        
		
	memcpy((void*)boardIn, inputs_AFU, sizeof(float)*(((value_times*2)+10)*16));
        
        
        
        
	
	    btUnsignedInt        cl;               // Loop counter. Cache-Line number.
	int                  tres;              // If many errors in buffer, only dump a limited number
	btInt                res = 0;
	ostringstream        oss("");          // Place to stash fancy strings
	btUnsigned32bitInt   tCacheLine[16];   // Temporary cacheline for various purposes
	CASSERT( sizeof(tCacheLine) == CL(1) );
		
        float *pu32 = reinterpret_cast<float*>(&pDestCL[0]); 
        int *iu32 = reinterpret_cast<int*>(&pDestCL[0]); 
        //pu32 = pu32+15*7;      
//high_resolution_clock::time_point t1 = high_resolution_clock::now(); 
//high_resolution_clock::time_point t3 = high_resolution_clock::now();      
	 //high_resolution_clock::time_point t4;		
//#####################################################################################################
//#####################################################################################################
//######################################## START 1ST COLLISION ########################################
//#####################################################################################################
//#####################################################################################################

	inputs_AFU[0] = 3.700012;

	memcpy((void*)boardIn, inputs_AFU, sizeof(float));
     
      volatile bt32bitInt done = pVAFU2_cntxt->Status & VAFU2_CNTXT_STATUS_DONE;
      int counter = 0;
      while (!done /*&& --count*/) {
		 //t4 = high_resolution_clock::now();
		 //auto durationtest = duration_cast<nanoseconds>( t4 - t3 ).count(); 
		 //if(durationtest >= 100000000){
			 //cout << "ERROR TIMEOUT\n";
			 //runOk = 0;
			 //break;
		 //}
         done = pVAFU2_cntxt->Status & VAFU2_CNTXT_STATUS_DONE;
		 cout << "";
      }

      runOk = 1;
//#####################################################################################################
//#####################################################################################################		
//######################################## END 1ST COLLISION ##########################################
//#####################################################################################################
//#####################################################################################################	
//high_resolution_clock::time_point t2 = high_resolution_clock::now();
//auto duration = duration_cast<nanoseconds>( t2 - t1 ).count(); 
//timescollideavg = timescollideavg + duration;
if(runOk == 1){
	 /*cout << "ADDR1: " << addr1 << "\n";
		timescollideavg = timescollideavg + duration;
		cout << "TIMES " << duration << "\n";*/ 
		int countdataout = 0;
       for(int i = 0;i< (addr1+2)*16*8;i++){
            if((i+1)%8 == 0){
                cout << "float debug: " << *pu32 << "\n";
                cout << "int debug: "   << *iu32 << "\n";
            }else {
                if(i%8 == 0){ 
                    cout << "\naddress " << countdataout << "\n";
                    countdataout++;
                }
                cout << *pu32 << "\n";
                
            }
            ++pu32;
            ++iu32;
            
        }
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
   return runOk;
}

// We must implement the IServiceClient interface (IServiceClient.h):

// <begin IServiceClient interface>
void dCollideSpheres::serviceAllocated(IBase               *pServiceBase,
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

   MSG("Service Allocated");

   // Allocate Workspaces needed. ASE runs more slowly and we want to watch the transfers,
   //   so have fewer of them.
#if defined ( ASEAFU )
#define LB_BUFFER_SIZE CL(16000)
#else
#define LB_BUFFER_SIZE MB(8)
#endif

   m_SPLService->WorkspaceAllocate(sizeof(VAFU2_CNTXT) + LB_BUFFER_SIZE + LB_BUFFER_SIZE,
      TransactionID());

}

void dCollideSpheres::serviceAllocateFailed(const IEvent &rEvent)
{
   IExceptionTransactionEvent * pExEvent = dynamic_ptr<IExceptionTransactionEvent>(iidExTranEvent, rEvent);
   ERR("Failed to allocate a Service");
   ERR(pExEvent->Description());
   ++m_Result;
   m_Sem.Post(1);
}

 void dCollideSpheres::_DumpCL( void         *pCL,  // pointer to cache-line to print
                              ostringstream &oss)  // add it to this ostringstream
 {
    oss << std::hex << std::setfill('0') << std::uppercase;
    btUnsigned32bitInt *pu32 = reinterpret_cast<btUnsigned32bitInt*>(pCL);
    for( int i = 0; i < 2; ++i ) {
       oss << "0x" << std::setw(8) << *pu32 << " ";
       ++pu32;
    }
    oss << std::nouppercase;
 }  // _DumpCL

void dCollideSpheres::serviceFreed(TransactionID const &rTranID)
{
   MSG("Service Freed");
   // Unblock Main()
   m_Sem.Post(1);
}

// <ISPLClient>
void dCollideSpheres::OnWorkspaceAllocated(TransactionID const &TranID,
                                  btVirtAddr           WkspcVirt,
                                  btPhysAddr           WkspcPhys,
                                  btWSSize             WkspcSize)
{
   AutoLock(this);

   m_pWkspcVirt = WkspcVirt;
   m_WkspcSize = WkspcSize;

   INFO("Got Workspace");         // Got workspace so unblock the Run() thread
   m_Sem.Post(1);
}

void dCollideSpheres::OnWorkspaceAllocateFailed(const IEvent &rEvent)
{
   IExceptionTransactionEvent * pExEvent = dynamic_ptr<IExceptionTransactionEvent>(iidExTranEvent, rEvent);
   ERR("OnWorkspaceAllocateFailed");
   ERR(pExEvent->Description());
   ++m_Result;
   m_Sem.Post(1);
}

void dCollideSpheres::OnWorkspaceFreed(TransactionID const &TranID)
{
   ERR("OnWorkspaceFreed");
   // Freed so now Release() the Service through the Services IAALService::Release() method
   (dynamic_ptr<IAALService>(iidService, m_pAALService))->Release(TransactionID());
}

void dCollideSpheres::OnWorkspaceFreeFailed(const IEvent &rEvent)
{
   IExceptionTransactionEvent * pExEvent = dynamic_ptr<IExceptionTransactionEvent>(iidExTranEvent, rEvent);
   ERR("OnWorkspaceAllocateFailed");
   ERR(pExEvent->Description());
   ++m_Result;
   m_Sem.Post(1);
}

/// CMyApp Client implementation of ISPLClient::OnTransactionStarted
void dCollideSpheres::OnTransactionStarted( TransactionID const &TranID,
                                   btVirtAddr           AFUDSMVirt,
                                   btWSSize             AFUDSMSize)
{
   INFO("Transaction Started");
   m_AFUDSMVirt = AFUDSMVirt;
   m_AFUDSMSize =  AFUDSMSize;
   m_Sem.Post(1);
}
/// CMyApp Client implementation of ISPLClient::OnContextWorkspaceSet
void dCollideSpheres::OnContextWorkspaceSet( TransactionID const &TranID)
{
   INFO("Context Set");
   m_Sem.Post(1);
}
/// CMyApp Client implementation of ISPLClient::OnTransactionFailed
void dCollideSpheres::OnTransactionFailed( const IEvent &rEvent)
{
   IExceptionTransactionEvent * pExEvent = dynamic_ptr<IExceptionTransactionEvent>(iidExTranEvent, rEvent);
   MSG("Runtime AllocateService failed");
   MSG(pExEvent->Description());
   m_bIsOK = false;
   ++m_Result;
   m_AFUDSMVirt = NULL;
   m_AFUDSMSize =  0;
   ERR("Transaction Failed");
   m_Sem.Post(1);
}
/// CMyApp Client implementation of ISPLClient::OnTransactionComplete
void dCollideSpheres::OnTransactionComplete( TransactionID const &TranID)
{
   m_AFUDSMVirt = NULL;
   m_AFUDSMSize =  0;
   INFO("Transaction Complete");
   m_Sem.Post(1);
}
/// CMyApp Client implementation of ISPLClient::OnTransactionStopped
void dCollideSpheres::OnTransactionStopped( TransactionID const &TranID)
{
   m_AFUDSMVirt = NULL;
   m_AFUDSMSize =  0;
   INFO("Transaction Stopped");
   m_Sem.Post(1);
}
void dCollideSpheres::serviceEvent(const IEvent &rEvent)
{
   ERR("unexpected event 0x" << hex << rEvent.SubClassID());
}
// <end IServiceClient interface>

/// @} group dCollideSpheresSample


//=============================================================================
// Name: main
// Description: Entry point to the application
// Inputs: none
// Outputs: none
// Comments: Main initializes the system. The rest of the example is implemented
//           in the objects.
//=============================================================================
int main(int argc, char *argv[])
{
btInt Result;
uint32_t times_run_aux = 1073741824;
uint32_t times_run_aux2;
uint32_t times_run = 4;
int num_modules = 10;
int compensation = 0;
int numlines = 20;
int address1 = 2;
int address2 = 4;

if(argc > 1){
	numlines = atoi(argv[1]);
	cout << "numlines = " << numlines << "\n";
}else {
	numlines = 20;
        cout << "numlines = " << numlines << "\n";

}
if(argc > 2){
	address1 = atoi(argv[2]);
	cout << "address1 = " << address1 << "\n";		
}
else{
	address1 = 2;
        cout << "address1 = " << address1 << "\n";

}
if(argc > 3){
	address2 = atoi(argv[3]);
	cout << "address2 = " << address2 << "\n";
}
else{
	address2 = 2;
        cout << "address2 = " << address2 << "\n";

}

while (Result != 1) {
RuntimeClient  runtimeClient;
        dCollideSpheres theApp(&runtimeClient, argv[1]);
        
        if(!runtimeClient.isOK()){
            ERR("Runtime Failed to Start");
            exit(1);
        }
		Result = theApp.run(numlines,address1,address2);	
	
}
   return Result;
   
   
  
/*ofstream myfile;
  myfile.open ("averages.txt", ofstream::out | ofstream::app);
  
int Result;
uint32_t times_run_aux = 1073741824;
uint32_t times_run_aux2;
uint32_t times_run = 32;
int num_modules = 16;
int compensation = 0;
int num_tests = 7;
long long int previousValue = 0;
long long int subValue = 0;
int numlines = 500;
int address1 = 1;
int address2 = 100;
myfile << "number of collision\t" << "average total of collisions (7 times)\t" << "average per collison \n";
myfile.close();
for(int j = 1; j <= times_run; j=j+1){
	myfile.open ("averages.txt", ofstream::out | ofstream::app);
    for(int i = 0; i< num_tests; i++){
        
		while (Result != 1) {
		RuntimeClient  runtimeClient;
				dCollideSpheres theApp(&runtimeClient, argv[1]);
				
				if(!runtimeClient.isOK()){
					ERR("Runtime Failed to Start");
					exit(1);
				}
				Result = theApp.run(numlines,j,address2);	
			
		}
		Result = 0;		
    }
		timescollideavg = timescollideavg/num_tests;
		timescollideavg = timescollideavg - compensation;
		float avgcollision = (timescollideavg)/(j*16);
		subValue = timescollideavg - previousValue;
		//cout << "\nAVERAGE : " << timescollideavg  << " AVG collision time: " << avgcollision << "\n";;
		myfile << j*16 << "\t" << timescollideavg  << "\t" << avgcollision << "\t" << subValue << "\n";
		myfile.close();
		previousValue = timescollideavg;
		timescollideavg = 0;
    
	
}   
   
return Result;*/
   
}

