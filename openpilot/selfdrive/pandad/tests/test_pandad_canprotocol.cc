#include <climits>

#include "common/tests/native_test.h"
#include "openpilot/cereal/messaging/messaging.h"
#include "selfdrive/pandad/panda.h"

struct PandaTest : public Panda {
  PandaTest(int can_list_size, cereal::PandaState::PandaType hw_type);
  void test_can_send();
  void test_can_recv(uint32_t chunk_size = 0);
  void test_chunked_can_recv();

  std::map<int, std::string> test_data;
  int can_list_size = 0;
  int total_pakets_size = 0;
  MessageBuilder msg;
  capnp::List<cereal::CanData>::Reader can_data_list;
};

PandaTest::PandaTest(int can_list_size_, cereal::PandaState::PandaType hw_type_) : can_list_size(can_list_size_), Panda() {
  this->hw_type = hw_type_;
  int data_limit = ((hw_type == cereal::PandaState::PandaType::RED_PANDA) ? std::size(dlc_to_len) : 8);
  // prepare test data
  for (int i = 0; i < data_limit; ++i) {
    int data_len = dlc_to_len[i];
    std::string bytes(data_len, '\0');
    for (int j = 0; j < data_len; ++j) bytes[j] = static_cast<char>((i * 31 + j) & 0xff);
    test_data[data_len] = bytes;
  }

  // generate can messages for this panda
  auto can_list = msg.initEvent().initSendcan(can_list_size);
  for (uint8_t i = 0; i < can_list_size; ++i) {
    auto can = can_list[i];
    uint32_t id = i % data_limit;
    const std::string &dat = test_data[dlc_to_len[id]];
    can.setAddress(i);
    can.setSrc(i % 3);
    can.setDat(kj::ArrayPtr((uint8_t *)dat.data(), dat.size()));
    total_pakets_size += sizeof(can_header) + dat.size();
  }

  can_data_list = can_list.asReader();
}

void PandaTest::test_can_send() {
  std::vector<uint8_t> unpacked_data;
  this->pack_can_buffer(can_data_list, [&](uint8_t *chunk, size_t size) {
    unpacked_data.insert(unpacked_data.end(), chunk, &chunk[size]);
  });
  CHECK(unpacked_data.size() == total_pakets_size);

  int cnt = 0;
  for (int pos = 0, pckt_len = 0; pos < unpacked_data.size(); pos += pckt_len) {
    can_header header;
    memcpy(&header, &unpacked_data[pos], sizeof(can_header));
    const uint8_t data_len = dlc_to_len[header.data_len_code];
    pckt_len = sizeof(can_header) + data_len;

    CHECK(header.addr == cnt);
    CHECK(test_data.find(data_len) != test_data.end());
    const std::string &dat = test_data[data_len];
    CHECK(memcmp(dat.data(), &unpacked_data[pos + sizeof(can_header)], dat.size()) == 0);
    ++cnt;
  }
  CHECK(cnt == can_list_size);
}

void PandaTest::test_can_recv(uint32_t rx_chunk_size) {
  std::vector<can_frame> frames;
  this->pack_can_buffer(can_data_list, [&](uint8_t *data, uint32_t size) {
    if (rx_chunk_size == 0) {
      CHECK(this->unpack_can_buffer(data, size, frames));
    } else {
      this->receive_buffer_size = 0;
      uint32_t pos = 0;

      while (pos < size) {
        uint32_t chunk_size = std::min(rx_chunk_size, size - pos);
        memcpy(&this->receive_buffer[this->receive_buffer_size], &data[pos], chunk_size);
        this->receive_buffer_size += chunk_size;
        pos += chunk_size;

        CHECK(this->unpack_can_buffer(this->receive_buffer, this->receive_buffer_size, frames));
      }
    }
  });

  CHECK(frames.size() == can_list_size);
  for (int i = 0; i < frames.size(); ++i) {
    CHECK(frames[i].address == i);
    CHECK(test_data.find(frames[i].dat.size()) != test_data.end());
    const std::string &dat = test_data[frames[i].dat.size()];
    CHECK(memcmp(dat.data(), frames[i].dat.data(), dat.size()) == 0);
  }
}

void test_can_protocol() {
  for (auto hw_type : {cereal::PandaState::PandaType::DOS, cereal::PandaState::PandaType::RED_PANDA}) {
    for (int can_list_size : {1, 3, 5, 10, 30, 60, 100, 200}) {
      PandaTest send_test(can_list_size, hw_type);
      send_test.test_can_send();

      PandaTest receive_test(can_list_size, hw_type);
      receive_test.test_can_recv();

      PandaTest chunked_receive_test(can_list_size, hw_type);
      chunked_receive_test.test_can_recv(0x40);
    }
  }
}

// horizonpilot: GV60 preconditioning test mode. With precond_classic_bus0 set, bus 0 frames carry an explicit FD flag:
// 0xC7 classic, everything else CAN-FD. Other buses and the default mode leave the flag clear. unpack_can_buffer
// records the flag of 0xC7 frames on bus 0, separately for received frames and our own TX echoes.
struct PrecondTest : public Panda {
  void run() {
    struct Frame { uint32_t addr; uint8_t bus; uint8_t len; };
    const std::vector<Frame> frames = {{0xC7, 0, 8}, {0x2AA, 0, 32}, {0x1CF, 0, 8}, {0xC7, 2, 8}, {0x12A, 1, 16}};
    MessageBuilder msg;
    auto can_list = msg.initEvent().initSendcan(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
      std::string dat(frames[i].len, '\x01');
      can_list[i].setAddress(frames[i].addr);
      can_list[i].setSrc(frames[i].bus);
      can_list[i].setDat(kj::ArrayPtr((uint8_t *)dat.data(), dat.size()));
    }

    for (bool classic : {false, true}) {
      precond_classic_bus0 = classic;
      std::vector<uint8_t> packed;
      pack_can_buffer(can_list.asReader(), [&](uint8_t *chunk, size_t size) { packed.insert(packed.end(), chunk, &chunk[size]); });

      std::vector<bool> fd;
      for (size_t pos = 0; pos < packed.size(); pos += sizeof(can_header) + dlc_to_len[((can_header *)&packed[pos])->data_len_code]) {
        fd.push_back(((can_header *)&packed[pos])->reserved);
      }
      const std::vector<bool> expected = classic ? std::vector<bool>{false, true, true, false, false} : std::vector<bool>(5, false);
      CHECK(fd == expected);
    }

    // receive side: car's 0xC7 as classic, our echo as CAN-FD, a rejected frame ignored, 0xC7 on bus 2 ignored
    CHECK(precond_car_fd == -1);
    CHECK(precond_ours_fd == -1);
    auto rx = [&](uint8_t bus, bool fd_bit, bool returned, bool rejected) {
      uint8_t buf[sizeof(can_header) + 8] = {};
      can_header h = {};
      h.reserved = fd_bit; h.bus = bus; h.data_len_code = 8; h.returned = returned; h.rejected = rejected; h.addr = 0xC7;
      memcpy(buf, &h, sizeof(h));
      ((can_header *)buf)->checksum = calculate_checksum(buf, sizeof(buf));
      uint32_t size = sizeof(buf);
      std::vector<can_frame> out;
      CHECK(unpack_can_buffer(buf, size, out));
      CHECK(out.size() == 1);
    };
    rx(0, false, false, false);
    rx(0, true, true, false);
    CHECK(precond_car_fd == 0);
    CHECK(precond_ours_fd == 1);
    rx(0, true, false, true);   // rejected: not recorded
    rx(2, true, false, false);  // other bus: not recorded
    CHECK(precond_car_fd == 0);
  }
};

int main() {
  return run_native_test([] {
    test_can_protocol();
    PrecondTest().run();
  });
}
