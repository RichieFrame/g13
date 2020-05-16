//
// Created by khampf on 13-05-2020.
//

#include "logo.hpp"

#include <chrono>
#include <csignal>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <log4cpp/OstreamAppender.hh>
#include <memory>
#include <mutex>
#include <thread>

#include "helper.hpp"

#include "g13_keys.hpp"

#include "g13.hpp"
#include "g13_device.hpp"
#include "g13_manager.hpp"

namespace G13 {

// definitions
bool G13_Manager::running = true;
std::map<std::string, std::string> G13_Manager::stringConfigValues;
libusb_context *G13_Manager::libusbContext;
std::condition_variable G13_Manager::wakeup;
std::vector<G13::G13_Device *> G13_Manager::g13s;
libusb_hotplug_callback_handle G13_Manager::hotplug_cb_handle[3];
const int G13_Manager::class_id = LIBUSB_HOTPLUG_MATCH_ANY;

std::map<G13_KEY_INDEX, std::string> G13_Manager::g13_key_to_name;
std::map<std::string, G13_KEY_INDEX> G13_Manager::g13_name_to_key;
std::map<LINUX_KEY_VALUE, std::string> G13_Manager::input_key_to_name;
std::map<std::string, LINUX_KEY_VALUE> G13_Manager::input_name_to_key;

libusb_device **G13_Manager::devs;
std::string G13_Manager::logoFilename;

G13_Manager::G13_Manager() /* : libusbContext(nullptr), devs(nullptr)*/ {
  InitKeynames();
}

void G13_Manager::start_logging() {
  log4cpp::Appender *appender1 =
      new log4cpp::OstreamAppender("console", &std::cout);
  appender1->setLayout(new log4cpp::BasicLayout());
  log4cpp::Category &root = log4cpp::Category::getRoot();
  root.addAppender(appender1);

  // TODO: this is for later when --log_file is implemented
  //    log4cpp::Appender *appender2 = new log4cpp::FileAppender("default",
  //    "g13d-output.log"); appender2->setLayout(new log4cpp::BasicLayout());
  //    log4cpp::Category &sub1 =
  //    log4cpp::Category::getInstance(std::string("sub1"));
  //    sub1.addAppender(appender2);
}

// *************************************************************************
int G13_Manager::OpenAndAddG13(libusb_device *dev) {

  libusb_device_handle *handle;
  int error = libusb_open(dev, &handle);
  if (error != LIBUSB_SUCCESS) {
    G13_ERR("Error opening G13 device: "
            << G13_Device::DescribeLibusbErrorCode(error));
    return 1;
  }

  libusb_set_auto_detach_kernel_driver(handle, true);
  error = libusb_claim_interface(handle, 0);
  if (error != LIBUSB_SUCCESS) {
    G13_ERR("Cannot Claim Interface: "
            << G13_Device::DescribeLibusbErrorCode(error));
  }
  if (error == LIBUSB_ERROR_BUSY) {
    if (libusb_kernel_driver_active(handle, 0) == 1) {
      if (libusb_detach_kernel_driver(handle, 0) == 0) {
        G13_ERR("Kernel driver detached");
      }
      error = libusb_claim_interface(handle, 0);
      G13_ERR("Still cannot claim Interface: "
              << G13_Device::DescribeLibusbErrorCode(error));
    }
  }

  if (error == LIBUSB_SUCCESS) {
    G13_DBG("Interface successfully claimed");
    auto g13 = new G13_Device(dev, libusbContext, handle, g13s.size());
    g13s.push_back(g13);
    return 0;
  }

  libusb_release_interface(handle, 0);
  libusb_close(handle);
  return 1;
}

void G13_Manager::SetLogLevel(log4cpp::Priority::PriorityLevel lvl) {
  G13_OUT("set log level to " << lvl);
}

void G13_Manager::SetLogLevel(const std::string &level) {
  log4cpp::Category &root = log4cpp::Category::getRoot();
  try {
    auto numLevel = log4cpp::Priority::getPriorityValue(level);
    root.setPriority(numLevel);
  } catch (std::invalid_argument &e) {
    G13_ERR("unknown log level " << level);
  }
}

void G13_Manager::DiscoverG13s(libusb_device **devs, ssize_t count) {
  for (int i = 0; i < count; i++) {
    libusb_device_descriptor desc;
    int ret = libusb_get_device_descriptor(devs[i], &desc);
    if (ret != LIBUSB_SUCCESS) {
      G13_ERR("Failed to get device descriptor");
      return;
    }
    if (desc.idVendor == G13::G13_VENDOR_ID &&
        desc.idProduct == G13::G13_PRODUCT_ID) {
      OpenAndAddG13(devs[i]);
    }
  }
}

void G13_Manager::Cleanup() {
  G13_OUT("Cleaning up");
  for (auto handle : hotplug_cb_handle) {
    libusb_hotplug_deregister_callback(libusbContext, handle);
  }
  for (auto g13 : g13s) {
    g13->cleanup();
    delete g13;
  }
  libusb_exit(libusbContext);
}

void G13_Manager::InitKeynames() {

  int key_index = 0;

  // setup maps to let us convert between strings and G13 key names
  for (auto &name : G13::G13_KEY_STRINGS) {
    g13_key_to_name[key_index] = name;
    g13_name_to_key[name] = key_index;
    G13_DBG("mapping G13 " << name << " = " << key_index);
    key_index++;
  }

  // setup maps to let us convert between strings and linux key names
  for (auto &symbol : G13::G13_SYMBOLS) {
    auto keyname = std::string("KEY_" + std::string(symbol));

    int code = libevdev_event_code_from_name(EV_KEY, keyname.c_str());
    if (code < 0) {
      G13_ERR("No input event code found for " << keyname);
    } else {
      // TODO: this seems to map ok but the result is off
      // assert(keyname.compare(libevdev_event_code_get_name(EV_KEY,code)) ==
      // 0); linux/input-event-codes.h

      input_key_to_name[code] = symbol;
      input_name_to_key[symbol] = code;
      G13_DBG("mapping " << symbol << " " << keyname << "=" << code);
    }
  }

  // setup maps to let us convert between strings and linux button names
  for (auto &symbol : G13::G13_BTN_SEQ) {
    auto name = std::string("M" + std::string(symbol));
    auto keyname = std::string("BTN_" + std::string(symbol));
    int code = libevdev_event_code_from_name(EV_KEY, keyname.c_str());
    if (code < 0) {
      G13_ERR("No input event code found for " << keyname);
    } else {
      input_key_to_name[code] = name;
      input_name_to_key[name] = code;
      G13_DBG("mapping " << name << " " << keyname << "=" << code);
    }
  }
}

void G13_Manager::SignalHandler(int signal) {
  G13_OUT("Caught signal " << signal << " (" << strsignal(signal) << ")");
  running = false;
  wakeup.notify_all();
}

std::string G13_Manager::getStringConfigValue(const std::string &name) {
  try {
    return Helper::find_or_throw(stringConfigValues, name);
  } catch (...) {
    return "";
  }
}

void G13_Manager::setStringConfigValue(const std::string &name,
                                       const std::string &value) {
  G13_DBG("setStringConfigValue " << name << " = " << Helper::repr(value));
  stringConfigValues[name] = value;
}

std::string G13_Manager::MakePipeName(G13::G13_Device *d, bool is_input) {
  if (is_input) {
    std::string config_base = getStringConfigValue("pipe_in");
    if (!config_base.empty()) {
      if (d->id_within_manager() == 0) {
        return config_base;
      } else {
        return config_base + "-" + std::to_string(d->id_within_manager());
      }
    }
    return CONTROL_DIR + "g13-" + std::to_string(d->id_within_manager());
  } else {
    std::string config_base = getStringConfigValue("pipe_out");
    if (!config_base.empty()) {
      if (d->id_within_manager() == 0) {
        return config_base;
      } else {
        return config_base + "-" + std::to_string(d->id_within_manager());
      }
    }

    return CONTROL_DIR + "g13-" + std::to_string(d->id_within_manager()) +
           "_out";
  }
}

int LIBUSB_CALL G13_Manager::HotplugCallbackEnumerate(
    struct libusb_context *ctx, struct libusb_device *dev,
    libusb_hotplug_event event, void *user_data) {
  HotplugCallbackInsert(ctx, dev, event, user_data);
  for (auto g13 : g13s) {
    if (dev == g13->Device()) {
      SetupDevice(g13);
    }
  }
  return 0;
}

int LIBUSB_CALL G13_Manager::HotplugCallbackInsert(struct libusb_context *ctx,
                                                   struct libusb_device *dev,
                                                   libusb_hotplug_event event,
                                                   void *user_data) {
  G13_OUT("USB device connected");

  // Just make sure we have not been called multiple times
  for (auto g13 : g13s) {
    if (dev == g13->Device()) {
      return 0;
    }
  }

  // It's brand new!
  OpenAndAddG13(dev);
  // wakeup.notify_all();
  return 0; // Rearm
}

int LIBUSB_CALL G13_Manager::HotplugCallbackRemove(struct libusb_context *ctx,
                                                   struct libusb_device *dev,
                                                   libusb_hotplug_event event,
                                                   void *user_data) {

  G13_OUT("USB device disconnected");
  int i = 0;
  for (auto iter = g13s.begin(); (iter != g13s.end()); ++i) {
    if (dev == (*iter)->Device()) {
      G13_OUT("Closing device " << i);
      (*iter)->cleanup();
      iter = g13s.erase(iter);
    } else {
      iter++;
    }
  }
  return 0; // Rearm
}

G13::LINUX_KEY_VALUE G13_Manager::FindG13KeyValue(const std::string &keyname) {
  auto i = g13_name_to_key.find(keyname);
  if (i == g13_name_to_key.end()) {
    return G13::BAD_KEY_VALUE;
  }
  return i->second;
}

G13::LINUX_KEY_VALUE
G13_Manager::FindInputKeyValue(const std::string &keyname) {
  // if there is a KEY_ prefix, strip it off
  if (!strncmp(keyname.c_str(), "KEY_", 4)) {
    return FindInputKeyValue(keyname.c_str() + 4);
  }

  auto i = input_name_to_key.find(keyname);
  if (i == input_name_to_key.end()) {
    return G13::BAD_KEY_VALUE;
  }
  return i->second;
}

std::string G13_Manager::FindInputKeyName(G13::LINUX_KEY_VALUE v) {
  try {
    return Helper::find_or_throw(input_key_to_name, v);
  } catch (...) {
    return "(unknown linux key)";
  }
}

void G13_Manager::SetupDevice(G13::G13_Device *g13) {

  g13->register_context(libusbContext);
  if (!logoFilename.empty()) {
    g13->write_lcd_file(logoFilename);
  }

  G13_OUT("Active Stick zones ");
  g13->stick().dump(std::cout);

  std::string config_fn = getStringConfigValue("config");
  if (!config_fn.empty()) {
    G13_OUT("config_fn = " << config_fn);
    g13->read_config_file(config_fn);
  }
}

std::string G13_Manager::FindG13KeyName(G13::G13_KEY_INDEX v) {
  try {
    return Helper::find_or_throw(g13_key_to_name, v);
  } catch (...) {
    return "(unknown G13 key)";
  }
}

void G13_Manager::DisplayKeys() {
  typedef std::map<std::string, int> mapType;
  G13_OUT("Known keys on G13:");
  G13_OUT(Helper::map_keys_out(g13_name_to_key));

  G13_OUT("Known keys to map to:");
  G13_OUT(Helper::map_keys_out(input_name_to_key));
}

int G13_Manager::Run() {
  DisplayKeys();

  ssize_t cnt;
  int error;

  error = libusb_init(&libusbContext);
  if (error != LIBUSB_SUCCESS) {
    G13_ERR("libusb initialization error: "
            << G13_Device::DescribeLibusbErrorCode(error));
    Cleanup();
    return EXIT_FAILURE;
  }
  libusb_set_option(libusbContext, LIBUSB_OPTION_LOG_LEVEL, 3);

  if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
    cnt = libusb_get_device_list(libusbContext, &devs);
    if (cnt < 0) {
      G13_ERR("Error while getting device list");
      Cleanup();
      return EXIT_FAILURE;
    }

    DiscoverG13s(devs, cnt);
    libusb_free_device_list(devs, 1);
    G13_OUT("Found " << g13s.size() << " G13s");
    if (g13s.empty()) {
      G13_ERR("Unable to open any device");
      Cleanup();
      return EXIT_FAILURE;
    }

    for (auto &g13 : g13s) {
      SetupDevice(g13);
    }
  } else {
    ArmHotplugCallbacks();
  }

  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  do {
    if (g13s.empty()) {
      G13_OUT("Waiting for device to show up ...");
      error = libusb_handle_events(libusbContext);
      G13_DBG("USB Event wakeup");
      if (error != LIBUSB_SUCCESS) {
        G13_ERR("Error: " << G13_Device::DescribeLibusbErrorCode(error));
      } else {
        for (auto g13 : g13s) {
          // This can not be done from the event handler (will give LIBUSB_ERROR_BUSY)
          SetupDevice(g13);
        }
      }
    }

    // Main loop
    for (auto g13 : g13s) {
      int status = g13->read_keys();
      g13->read_commands();
      if (status < 0) {
        running = false;
      }
    }
  } while (running);

  Cleanup();
  G13_OUT("Exit");
  return EXIT_SUCCESS;
}

void G13_Manager::ArmHotplugCallbacks() {
  int error;

  G13_DBG("Registering USB hotplug callbacks");

  // For currently attached devices
  error = libusb_hotplug_register_callback(
      libusbContext, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED,
      LIBUSB_HOTPLUG_ENUMERATE, G13_VENDOR_ID, G13_PRODUCT_ID, class_id,
      HotplugCallbackEnumerate, nullptr, &hotplug_cb_handle[0]);
  if (error != LIBUSB_SUCCESS) {
    G13_ERR("Error registering hotplug enumeration callback: "
            << G13_Device::DescribeLibusbErrorCode(error));
  }

  // For future devices
  error = libusb_hotplug_register_callback(
      libusbContext, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED,
      LIBUSB_HOTPLUG_NO_FLAGS, G13_VENDOR_ID, G13_PRODUCT_ID, class_id,
      HotplugCallbackInsert, nullptr, &hotplug_cb_handle[1]);
  if (error != LIBUSB_SUCCESS) {
    G13_ERR("Error registering hotplug insertion callback: "
            << G13_Device::DescribeLibusbErrorCode(error));
  }

  // For disconnected devices
  error = libusb_hotplug_register_callback(
      libusbContext, LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_NO_FLAGS,
      G13_VENDOR_ID, G13_PRODUCT_ID, class_id, HotplugCallbackRemove, nullptr,
      &hotplug_cb_handle[2]);
  if (error != LIBUSB_SUCCESS) {
    G13_ERR("Error registering hotplug removal callback: "
            << G13_Device::DescribeLibusbErrorCode(error));
  }
}

/*
    libusb_context *G13_Manager::getCtx() {
        return libusbContext;
    }

    void G13_Manager::setCtx(libusb_context *newLibusbContext) {
        libusbContext = newLibusbContext;
    }
*/

/*
    const std::string &G13_Manager::getLogoFilename() {
        return logoFilename;
    }
*/

void G13_Manager::setLogoFilename(const std::string &newLogoFilename) {
  logoFilename = newLogoFilename;
}

} // namespace G13