#pragma once

#include <array>
#include <atomic>
#include <stdexcept>
#include <string_view>
#include <thread>
#include "Constants.h"
#include "SubsystemBase.h"

/**
 * A base class for subsystems with controllers.
 *
 * State, Inputs, and Outputs indices should be specified what they represent in
 * the derived class.
 *
 * @tparam States the number of state estimates in the state vector
 * @tparam Inputs the number of control inputs in the input vector
 * @tparam Outputs the number of local outputs in the output vector
 */
template <int States, int Inputs, int Outputs>
class ControlledSubsystemBase : public SubsystemBase {
    public:
        /**
         * Constructs a ControlledSubsystemBase.
         *
         * @param controllerName Name of the controller log file.
         * @param stateLabels    Labels for states each consisting of its name and unit.
         * @param inputLabels    Labels for inputs each consisting of its name and unit.
         * @param outputLabels   Labels for outputs each consisting of its name and unit.
         * @param isTesting      Whether or not the robot should log info.
         */
        ControlledSubsystemBase(
                std::string_view controllerName,
//                const std::array<ControllerLabel, States>& stateLabels,
//                const std::array<ControllerLabel, Inputs>& inputLabels,
//                const std::array<ControllerLabel, Outputs>& outputLabels,
                bool isTesting = false)
                :  m_isTesting(isTesting) {
            m_entryThreadRunning = true;
            m_entryThread = std::thread{[=] { EntryThreadMain(); }};
        }

        /**
         * Move constructor.
         */
        ControlledSubsystemBase(ControlledSubsystemBase&&) = default;

        /**
         * Move assignment operator.
         */
        ControlledSubsystemBase& operator=(ControlledSubsystemBase&&) = default;

        ~ControlledSubsystemBase() override {
            m_entryThreadRunning = false;
           // m_entryQueue.emplace();
            m_entryThread.join();
        }

        /**
         * Enables the control loop.
         */
        void Enable() {
            // m_lastTime is reset so that a large time delta isn't generated from
            // Update() not being called in a while.
            //m_lastTime = 0;  //TODO::need to get from system time.
            m_isEnabled = true;
        }

        /**
         * Disables the control loop.
         */
        void Disable() { m_isEnabled = false; }

        /**
         * Returns true if the control loop is enabled.
         */
        bool IsEnabled() const { return m_isEnabled; }

        /**
         * Returns true if the team needs logging information
         */
        bool IsTesting() const { return m_isTesting; }

        /**
         * Returns the most recent timestep.
         */
//        units::second_t GetDt() const { return m_dt; }

        /**
         * Runs periodic observer and controller update.
         */
        virtual void ControllerPeriodic() = 0;

        /**
         * Computes current timestep's dt.
         */
        void UpdateDt() {
//            m_nowBegin =  frc::Timer::GetFPGATimestamp();
//            m_nowBegin = 0;
//            m_dt = m_nowBegin - m_lastTime;

//            if (m_dt == 0_s) {
//                m_dt = Constants::kControllerPeriod;
//                fmt::print(stderr, "ERROR @ t = {}: dt = 0\n", m_nowBegin);
//            }
//
//            // Clamp spikes in scheduling latency
//            if (m_dt > 10_ms) {
//                m_dt = Constants::kControllerPeriod;
//            }
        }

        /**
         * Logs the current controller information.
         *
         * @param r The references for this timestep.
         * @param x The states for this timestep.
         * @param u The inputs for this timestep.
         * @param y The measurements for this timestep.
         */
//        void Log(const Eigen::Vector<double, States>& r,
//                 const Eigen::Vector<double, States>& x,
//                 const Eigen::Vector<double, Inputs>& u,
//                 const Eigen::Vector<double, Outputs>& y) {
//            m_entryQueue.emplace(m_nowBegin, frc::Timer::GetFPGATimestamp(), r, x,
//                                 u, y);
//            m_lastTime = m_nowBegin;
//        }

    private:

//        units::second_t m_dt = Constants::kControllerPeriod;
        bool m_isEnabled = false;
        bool m_isTesting = false;

//        wpi::ConcurrentQueue<LogEntry> m_entryQueue;
        std::atomic<bool> m_entryThreadRunning{false};
        std::thread m_entryThread;

        void EntryThreadMain() {
        }
};