#ifndef ETHERCATMASTER_HPP
#define ETHERCATMASTER_HPP

#include <string>
#include "MailboxMessage.hpp"
#include "ProcessBuffer.hpp"
#include <memory>
#include <functional>
#include <vector>

namespace youbot {

  /// <summary>
  /// Abstract class to wrap the high level functionality of EtherCAT Master drivers.
  /// </summary>
  class EtherCATMaster {
  public:
    /** Currently only to types are implemented: PHYSICAL means SOEM and VIRTUAL is for simulated uses */
    enum Type {
      PHYSICAL, ///< SOEM-based implementation
      VIRTUAL ///< for simulations
    };

    /// <summary>
    /// Return the type of the intialized EtherCAT Master
    /// </summary>
    /// <returns></returns>
    virtual Type GetType() const = 0;

    ///< Desctructor
    ~EtherCATMaster();

    enum class MailboxStatus : uint8_t {
      INITIALIZED = 0,
      SENT_SUCCESSFUL = 2,
      RECEIVED_SUCCESSFUL = 4
    };

    /// <summary>
    /// Copy the content of processbuffer of the given slave into "buff"
    /// </summary>
    /// <param name="buff"> The content of the processbuffer of the given slave is copied into </param>
    /// <param name="slaveNumber"> Identifier of the slave 0..(N-1) </param>
    virtual void GetProcessMsg(ProcessBuffer& buff, uint8_t slaveNumber) const = 0;

    /// <summary>
    /// Set the useful size of recieved process messages to be copyied (by default zero)
    /// </summary>
    /// <param name="size"> Number of bytes </param>
    /// <param name="slaveNumber"> Identifier of the slave 0..(N-1) </param>
    virtual void SetProcessFromSlaveSize(uint8_t size, uint8_t slaveNumber) = 0;

    /// <summary>
    /// Prepare the buffer to send out to slave of slavenumber
    /// </summary>
    /// <param name="buffer"> The buffer to be copid into the processbuffer of the given slave </param>
    /// <param name="slaveNumber"> Identifier of the slave 0..(N-1) </param>
    /// <returns> The sent buffer size </returns>
    virtual int SetProcessMsg(const ProcessBuffer& buffer, uint8_t slaveNumber) = 0;

    /// <summary>
    /// Send out the prepared process buffers, read the buffers of the slaves into the "fromSaves" buffers
    /// </summary>
    virtual void ExchangeProcessMsg() = 0;

    /// <summary>
    ///   Send out a mailbox message, the returned value are available in the shared_ptr
    /// </summary>
    /// <param name="ptr"> Contains the sent and the recieved message </param>
    /// <returns> Describes if send and receive is successful </returns>
    virtual MailboxStatus SendMailboxMessage(MailboxMessage::Ptr ptr) = 0;

    /// <summary>
    /// Function to get the number of the connected etherCAT slaves
    /// </summary>
    /// <returns> The number of connected EtherCAT slaves </returns>
    virtual int getSlaveNum() const = 0; // cnt: 0..N-1

    /// <summary>
    /// Get name of a given slave
    /// </summary>
    /// <param name="cnt"> slavenumber 0..(N-1)</param>
    /// <returns> The name of the slave </returns>
    virtual std::string getSlaveName(int cnt) const = 0; // cnt: 0..N-1

    typedef std::shared_ptr<EtherCATMaster> Ptr;

#ifndef _ONLY_VIRTUAL_ROBOT
    /// <summary>
    /// Function to construct a SOEM EtherCAT Master instance
    /// </summary>
    /// <param name="adapterName"> the etherNET adress name </param>
    /// <returns> shared_ptr of the EtherCAT Master instance </returns>
    static Ptr CreatePhysical(const std::string& adapterName);
#endif

    /// <summary>
    /// Function to construct a SOEM EtherCAT Master instance
    /// </summary>
    /// <returns> shared_ptr of the EtherCAT Master instance </returns>
    static Ptr CreateVirtual();

    /// <summary>
    /// Register a callback that must be performed after ExchangeProcessMsg-s, used to read out returned buffers
    /// </summary>
    /// <param name="in"></param>
    void RegisterAfterExchangeCallback(std::function<void(void)> in);

  protected:
    /// <summary>
    /// Calls the register callbacks - must be used in the end of ExchangeProcessMsg implementations
    /// </summary>
    void _callAfterExchangeCallbacks(); // must be called at the end of the exchange

  private:
    std::vector<std::function<void(void)>> afterExchangeCallbacks; ///< Stored callbacks
  };
}
#endif
