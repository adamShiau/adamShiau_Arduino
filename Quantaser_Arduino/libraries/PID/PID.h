//==============Useful define variable it is suggest to copy this define to main program and remark these
#define PIDDEBUGFLAG 1500 // User choose how many times to run compute and show the paramter in Serail once
// #define PIDDEBUGFLAG01

#ifdef PIDDEBUG
	#define DEBUGFLAG01 PIDDEBUG
#else
#endif
class PID
{
	private:
		long long  g_p_limit, g_i_limit, g_errorlimit;//, g_errorsum;
		//long long g_errorsum;
//        unsigned int g_index; 
	public:
		PID();		
//		void Init(long long ,long long ,long long , unsigned char );
		void Init(long long ,long long ,unsigned char, unsigned char , unsigned char );
//		void Init(long long ,long long ,long long );
		long Compute(bool, long, unsigned char, unsigned char, unsigned char);
		void showParameter();
		
		long long g_errorsum; //Adam@1112, change to public
		long g_i_term, g_p_term;
		///////add to show parameters////////
		
		long g_errin, g_out;
		unsigned char g_kp, g_ki, g_ls, g_errgain;
		unsigned int g_index;
};
