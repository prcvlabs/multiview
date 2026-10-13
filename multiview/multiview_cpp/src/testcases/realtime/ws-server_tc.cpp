#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"

#include "perceive/io/json-io.hpp"
#include "perceive/pipeline/realtime/ws-server.hpp"

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include <thread>

namespace perceive::pipeline::realtime
{
namespace beast     = boost::beast;
namespace http      = boost::beast::http;
namespace websocket = boost::beast::websocket;
namespace net       = boost::asio;
using tcp           = boost::asio::ip::tcp;
using namespace std::chrono_literals;

// -------------------------------------------------------------------- endpoint
//
static tcp::endpoint endpoint(uint16_t port)
{
   return tcp::endpoint(net::ip::make_address("127.0.0.1"), port);
}

// -------------------------------------------------------------------- http-get
//
static http::response<http::string_body>
http_request(uint16_t port, http::verb verb, const string& target)
{
   net::io_context ioc;
   beast::tcp_stream stream(ioc);
   stream.connect(endpoint(port));
   http::request<http::string_body> req{verb, target, 11};
   req.set(http::field::host, "127.0.0.1");
   http::write(stream, req);
   beast::flat_buffer buffer;
   http::response<http::string_body> res;
   http::read(stream, buffer, res);
   return res;
}

// -------------------------------------------------------------------- wait-for
//
template<typename F> static bool wait_for(F&& is_done)
{
   const auto t1 = std::chrono::steady_clock::now() + 5s;
   while(!is_done()) {
      if(std::chrono::steady_clock::now() > t1) return false;
      std::this_thread::sleep_for(1ms);
   }
   return true;
}

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("WsServer", "[realtime_ws]")
{
   WsServer::Params params;
   params.address = "127.0.0.1";
   params.port    = 0; // any free port

   auto health = []() {
      Json::Value o{Json::objectValue};
      o["status"] = "ok";
      return o;
   };
   auto metrics = []() { return "multiview_realtime_slots_busy 1\n"s; };

   CATCH_SECTION("health-and-metrics")
   {
      WsServer server(params, health, metrics);
      CATCH_REQUIRE(server.start());
      CATCH_REQUIRE(server.port() > 0);

      const auto res_h
          = http_request(server.port(), http::verb::get, "/v1/health");
      CATCH_REQUIRE(res_h.result() == http::status::ok);
      CATCH_REQUIRE(res_h[http::field::content_type] == "application/json");
      CATCH_REQUIRE(parse_json(res_h.body())["status"].asString() == "ok");

      const auto res_m
          = http_request(server.port(), http::verb::get, "/v1/metrics");
      CATCH_REQUIRE(res_m.result() == http::status::ok);
      CATCH_REQUIRE(res_m.body() == metrics());

      CATCH_REQUIRE(
          http_request(server.port(), http::verb::get, "/v1/nope").result()
          == http::status::not_found);
      CATCH_REQUIRE(
          http_request(server.port(), http::verb::post, "/v1/health").result()
          == http::status::method_not_allowed);
      server.stop();
   }

   CATCH_SECTION("stream-in-order-then-close")
   {
      WsServer server(params, health, metrics);
      CATCH_REQUIRE(server.start());

      net::io_context ioc;
      websocket::stream<tcp::socket> ws(ioc);
      ws.next_layer().connect(endpoint(server.port()));
      ws.handshake("127.0.0.1", "/v1/stream");
      CATCH_REQUIRE(server.n_clients() == 1);

      constexpr int n = 100;
      for(int i = 0; i < n; ++i)
         server.broadcast(format("{{\"type\":\"frame\",\"frame\":{}}}", i));

      beast::flat_buffer buffer;
      for(int i = 0; i < n; ++i) {
         ws.read(buffer);
         CATCH_REQUIRE(ws.got_text());
         CATCH_REQUIRE(beast::buffers_to_string(buffer.data())
                       == format("{{\"type\":\"frame\",\"frame\":{}}}", i));
         buffer.consume(buffer.size());
      }

      // `stop` closes the client once it has its lines
      const auto t0 = std::chrono::steady_clock::now();
      std::thread stopper([&server]() { server.stop(); });
      beast::error_code ec;
      ws.read(buffer, ec);
      stopper.join();
      CATCH_REQUIRE(ec == websocket::error::closed);
      CATCH_REQUIRE(std::chrono::steady_clock::now() - t0 < 1s);
      CATCH_REQUIRE(server.n_sent() == n);
      CATCH_REQUIRE(server.n_dropped() == 0);
      CATCH_REQUIRE(server.n_clients() == 0);
   }

   CATCH_SECTION("greeting-first")
   {
      WsServer server(params, health, metrics);
      CATCH_REQUIRE(server.start());

      auto connect = [&](websocket::stream<tcp::socket>& ws) {
         ws.next_layer().connect(endpoint(server.port()));
         ws.handshake("127.0.0.1", "/v1/stream");
      };
      auto read = [](websocket::stream<tcp::socket>& ws) {
         beast::flat_buffer buffer;
         ws.read(buffer);
         return beast::buffers_to_string(buffer.data());
      };

      net::io_context ioc;
      websocket::stream<tcp::socket> early(ioc);
      websocket::stream<tcp::socket> late(ioc);
      connect(early);
      server.set_greeting("header");
      server.broadcast("a");
      CATCH_REQUIRE(read(early) == "header");
      CATCH_REQUIRE(read(early) == "a");

      connect(late);
      server.broadcast("b");
      CATCH_REQUIRE(read(late) == "header");
      CATCH_REQUIRE(read(late) == "b");
      CATCH_REQUIRE(read(early) == "b");

      std::thread stopper([&server]() { server.stop(); });
      beast::flat_buffer buffer;
      beast::error_code ec;
      late.read(buffer, ec);
      early.read(buffer, ec);
      stopper.join();
   }

   CATCH_SECTION("slow-client-drops-oldest")
   {
      params.client_queue    = 4;
      params.close_timeout_s = 0.5;
      WsServer server(params, health, metrics);
      CATCH_REQUIRE(server.start());

      // A client that never reads: its socket buffers fill, then its queue
      net::io_context ioc;
      websocket::stream<tcp::socket> ws(ioc);
      ws.next_layer().connect(endpoint(server.port()));
      ws.handshake("127.0.0.1", "/v1/stream");

      constexpr int n = 64;
      const string line(1 << 20, 'x');
      for(int i = 0; i < n; ++i) server.broadcast(line);
      CATCH_REQUIRE(wait_for([&]() {
         return server.n_sent() + server.n_dropped() + 4 + 1 >= n
                and server.n_dropped() > 0;
      }));
      CATCH_REQUIRE(server.n_sent() + server.n_dropped() <= n);

      // Other requests are still answered
      CATCH_REQUIRE(
          http_request(server.port(), http::verb::get, "/v1/health").result()
          == http::status::ok);

      const auto t0 = std::chrono::steady_clock::now();
      server.stop(); // gives up on the client after `close_timeout_s`
      CATCH_REQUIRE(std::chrono::steady_clock::now() - t0 < 2s);
   }

   CATCH_SECTION("port-in-use")
   {
      WsServer first(params, health, metrics);
      CATCH_REQUIRE(first.start());
      params.port = first.port();
      WsServer second(params, health, metrics);
      CATCH_REQUIRE(!second.start());
   }
}

} // namespace perceive::pipeline::realtime
