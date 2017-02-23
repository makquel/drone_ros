#include "serialkey.h"

#include <iostream>
#include <list>
#include <iomanip>
#include <stdexcept>


#include <xsens/xstime.h>

#include "conio.h" // for non ANSI _kbhit() and _getch()



#include "mtiG.h"
/**
 * @brief Callback handler for incoming data packets from the device
 * @param maxBufferSize
 */
class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5) : m_maxNumberOfPacketsInBuffer(maxBufferSize), m_numberOfPacketsInBuffer(0)
#ifdef _MSC_VER
	{InitializeCriticalSection(&m_CriticalSection);}
	virtual ~CallbackHandler() throw() {DeleteCriticalSection(&m_CriticalSection);}
#else
	{
	  //create mutex attribute variable
	  pthread_mutexattr_t mAttr;

	  // setup recursive mutex for mutex attribute
	  pthread_mutexattr_settype(&mAttr, PTHREAD_MUTEX_RECURSIVE_NP);

	  // Use the mutex attribute to create the mutex
	  pthread_mutex_init(&m_CriticalSection, &mAttr);

	  // Mutex attribute can be destroy after initializing the mutex variable
	  pthread_mutexattr_destroy(&mAttr);

	}
	virtual ~CallbackHandler() throw() {pthread_mutex_destroy(&m_CriticalSection);}
#endif

	bool packetAvailable() const {Locker lock(*this); return m_numberOfPacketsInBuffer > 0;}
	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		Locker lock(*this);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	virtual void onDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		Locker lock(*this);
		assert(packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
		{
			(void)getNextPacket();
		}
		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
#ifdef _MSC_VER
	mutable CRITICAL_SECTION m_CriticalSection;
#else
	mutable pthread_mutex_t m_CriticalSection;
#endif
	struct Locker
	{
#ifdef _MSC_VER
		Locker(CallbackHandler const & self) : m_self(self) {EnterCriticalSection(&m_self.m_CriticalSection);}
		~Locker() throw() {LeaveCriticalSection(&m_self.m_CriticalSection);}
#else
		Locker(CallbackHandler const & self) : m_self(self) {pthread_mutex_lock(&m_self.m_CriticalSection);}
		~Locker() throw() {pthread_mutex_unlock(&m_self.m_CriticalSection);}
#endif
		CallbackHandler const & m_self;
	};
	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	std::list<XsDataPacket> m_packetBuffer;
};
