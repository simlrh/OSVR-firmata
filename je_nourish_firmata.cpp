// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>

// Generated JSON header file
#include "je_nourish_firmata_json.h"

// Library/third-party includes
#include "firmata.h"
#include "firmserial.h"

// Standard includes
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <memory>

// Anonymous namespace to avoid symbol collision
namespace {

	struct thread_data
	{
		std::mutex mutex;
		bool end;
		firmata::Base* firmata;
		std::string port;

		thread_data(std::string p) : port(p) {
			firmata = NULL;
			end = false;
		};

	};

	void thread_function(thread_data& data)
	{
		try {
			data.mutex.lock();

			firmata::FirmSerial* firmio = new firmata::FirmSerial(data.port);
			data.firmata = new firmata::Firmata<firmata::Base>(firmio);
			
			for (int i = 0; i < 6; i++) {
				data.firmata->reportAnalog(i, 1);
			}
			for (int i = 0; i < 14; i++) {
				data.firmata->pinMode(i, MODE_INPUT);
			}
			data.firmata->reportDigital(0, 1);
			data.firmata->reportDigital(1, 1);

			data.mutex.unlock();

			do {
				data.mutex.lock();
				if (data.end) break;
				data.firmata->parse();
				data.mutex.unlock();
			} while (1);

			data.mutex.unlock();
			delete data.firmata;
		}
		catch (firmata::IOException e) {
			std::cout << e.what() << std::endl;
		}
		catch (firmata::NotOpenException e) {
			std::cout << e.what() << std::endl;
		}
	}

	class FirmataDevice {
	public:
		FirmataDevice(OSVR_PluginRegContext ctx, std::string port, std::string firmwareName = "StandardFirmata.ino") : m_thread_data(port), m_valid(false) {
			std::cout << "Searching for " << firmwareName << " device on port " + port << "..." << std::endl;

			m_thread = new std::thread(thread_function, std::ref(m_thread_data));

			std::this_thread::sleep_for(std::chrono::milliseconds(3000));

			m_thread_data.mutex.lock();
			m_valid = m_thread_data.firmata != NULL && m_thread_data.firmata->ready() && m_thread_data.firmata->name.compare(firmwareName) == 0;
			m_thread_data.mutex.unlock();

			if (m_valid) {
				std::string deviceName = m_thread_data.firmata->name +
					"-" + std::to_string(m_thread_data.firmata->major_version) +
					"." + std::to_string(m_thread_data.firmata->minor_version);

				std::cout << "Found " << deviceName << std::endl;

				OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

				osvrDeviceAnalogConfigure(opts, &m_analog, 6);
				osvrDeviceButtonConfigure(opts, &m_button, 14);

				m_dev.initAsync(ctx, deviceName, opts);

				m_dev.sendJsonDescriptor(je_nourish_firmata_json);

				m_dev.registerUpdateCallback(this);
			}
			else {
				std::cout << "Not found" << std::endl;
			}

		}

		~FirmataDevice() {
			m_thread_data.mutex.lock();
			m_thread_data.end = true;
			m_thread_data.mutex.unlock();
			m_thread->join();
		}

		bool isValid() {
			return m_valid;
		}

		OSVR_ReturnCode update() {
			if (m_thread_data.end) {
				return OSVR_RETURN_SUCCESS;
			}
			m_thread_data.mutex.lock();

			if (m_thread_data.firmata != NULL && m_thread_data.firmata->ready()) {
				OSVR_AnalogState analog[6] = {
					m_thread_data.firmata->analogRead("A0"),
					m_thread_data.firmata->analogRead("A1"),
					m_thread_data.firmata->analogRead("A2"),
					m_thread_data.firmata->analogRead("A3"),
					m_thread_data.firmata->analogRead("A4"),
					m_thread_data.firmata->analogRead("A5")
				};

				osvrDeviceAnalogSetValues(m_dev, m_analog, analog, 6);

				OSVR_ButtonState buttons[14] = {
					m_thread_data.firmata->digitalRead(0),
					m_thread_data.firmata->digitalRead(1),
					m_thread_data.firmata->digitalRead(2),
					m_thread_data.firmata->digitalRead(3),
					m_thread_data.firmata->digitalRead(4),
					m_thread_data.firmata->digitalRead(5),
					m_thread_data.firmata->digitalRead(6),
					m_thread_data.firmata->digitalRead(7),
					m_thread_data.firmata->digitalRead(8),
					m_thread_data.firmata->digitalRead(9),
					m_thread_data.firmata->digitalRead(10),
					m_thread_data.firmata->digitalRead(11),
					m_thread_data.firmata->digitalRead(12),
					m_thread_data.firmata->digitalRead(13)
				};

				osvrDeviceButtonSetValues(m_dev, m_button, buttons, 14);
			}

			m_thread_data.mutex.unlock();
			return OSVR_RETURN_SUCCESS;
		}

	private:
		osvr::pluginkit::DeviceToken m_dev;
		OSVR_AnalogDeviceInterface m_analog;
		OSVR_ButtonDeviceInterface m_button;
		std::thread *m_thread;
		thread_data m_thread_data;
		bool m_valid;
	};

	class HardwareDetection {
	public:
		HardwareDetection() {}
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {
			std::vector<firmata::PortInfo> ports = firmata::FirmSerial::listPorts();

			for (auto port : ports) {
				bool used = false;
				for (auto used_port : m_ports) {
					if (used_port.compare(port.port) == 0) {
						used = true;
						break;
					}
				}
				if (used) continue;

				FirmataDevice* device = new FirmataDevice(ctx, port.port);

				if (device->isValid()) {
					osvr::pluginkit::registerObjectForDeletion(ctx, device);
					m_ports.push_back(port.port);
				}
				else {
					delete device;
				}
			}
			return OSVR_RETURN_SUCCESS;
		}
	private:
		std::vector<std::string> m_ports;
	};
} // namespace

OSVR_PLUGIN(je_nourish_firmata) {
	osvr::pluginkit::PluginContext context(ctx);

	context.registerHardwareDetectCallback(new HardwareDetection());

	return OSVR_RETURN_SUCCESS;
}
