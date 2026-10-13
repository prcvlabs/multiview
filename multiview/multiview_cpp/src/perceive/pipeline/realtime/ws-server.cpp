
#include "ws-server.hpp"

#include "perceive/io/json-io.hpp"

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include <condition_variable>
#include <deque>
#include <thread>

namespace perceive::pipeline::realtime
{
namespace beast     = boost::beast;
namespace http      = boost::beast::http;
namespace websocket = boost::beast::websocket;
namespace net       = boost::asio;
using tcp           = boost::asio::ip::tcp;
using error_code    = boost::system::error_code;
using line_ptr      = shared_ptr<const string>;
using request_type  = http::request<http::string_body>;

static constexpr std::chrono::seconds k_http_timeout{30};
static constexpr std::chrono::milliseconds k_accept_retry{100};
static constexpr const char* k_server_name = "multiview-realtime";

// ----------------------------------------------------------------------- Pimpl
// Everything but the atomics and `no_clients` belongs to the server's thread
// once it has started. The sessions refer to the Pimpl, so the members they
// touch are declared before `ioc`, whose destruction destroys them.
//
struct WsServer::Pimpl
{
   struct HttpSession;
   struct StreamSession;

   Params params;
   health_fn health;
   metrics_fn metrics;

   std::atomic<uint16_t> port      = 0;
   std::atomic<unsigned> n_clients = 0; // decremented under `padlock`
   std::atomic<uint64_t> n_sent    = 0;
   std::atomic<uint64_t> n_dropped = 0;
   std::atomic<bool> is_running    = false;
   std::mutex padlock;
   std::condition_variable no_clients;

   net::io_context ioc{1};
   tcp::acceptor acceptor{ioc};
   net::steady_timer retry_timer{ioc};
   std::optional<net::executor_work_guard<net::io_context::executor_type>> work;
   vector<std::weak_ptr<StreamSession>> sessions;
   line_ptr greeting = nullptr;
   bool is_stopping  = false;
   std::thread thread;

   void do_accept() noexcept;
   void on_stream(tcp::socket&& socket, request_type&& req) noexcept;
   void deliver(const line_ptr& line) noexcept;
   void close_all() noexcept;
   void client_gone() noexcept;
   void run() noexcept;
};

// ----------------------------------------------------------------- HttpSession
// One request, one response, then the connection is closed; or the request
// is a WebSocket upgrade at /v1/stream, and the socket becomes a
// `StreamSession`.
//
struct WsServer::Pimpl::HttpSession
    : public std::enable_shared_from_this<HttpSession>
{
   Pimpl& server;
   beast::tcp_stream stream;
   beast::flat_buffer buffer;
   request_type req;
   http::response<http::string_body> res;

   HttpSession(Pimpl& in_server, tcp::socket&& socket)
       : server(in_server)
       , stream(std::move(socket))
   {}

   // ---------------------------------------------------------------------- run
   //
   void run() noexcept
   {
      stream.expires_after(k_http_timeout);
      http::async_read(stream,
                       buffer,
                       req,
                       [self = shared_from_this()](error_code ec, size_t) {
                          self->on_read(ec);
                       });
   }

   // ------------------------------------------------------------------ on-read
   //
   void on_read(error_code ec) noexcept
   {
      if(ec) return; // the client went away, or timed out
      if(websocket::is_upgrade(req) and req.target() == "/v1/stream") {
         stream.expires_never();
         server.on_stream(stream.release_socket(), std::move(req));
         return;
      }
      respond();
   }

   // -------------------------------------------------------------------- reply
   //
   void reply(http::status status, const char* type, string body) noexcept
   {
      res.result(status);
      res.set(http::field::content_type, type);
      res.body() = std::move(body);
   }

   // ------------------------------------------------------------------ respond
   //
   void respond() noexcept
   {
      res.version(req.version());
      res.keep_alive(false);
      res.set(http::field::server, k_server_name);
      try {
         const auto target = req.target();
         if(req.method() != http::verb::get)
            reply(http::status::method_not_allowed, "text/plain", "GET only\n");
         else if(target == "/v1/health")
            reply(http::status::ok, "application/json", str(server.health()));
         else if(target == "/v1/metrics")
            reply(http::status::ok,
                  "text/plain; version=0.0.4",
                  server.metrics());
         else
            reply(http::status::not_found, "text/plain", "not found\n");
      } catch(std::exception& e) {
         reply(http::status::internal_server_error, "text/plain", e.what());
      }
      res.prepare_payload();

      http::async_write(
          stream, res, [self = shared_from_this()](error_code, size_t) {
             error_code ec;
             self->stream.socket().shutdown(tcp::socket::shutdown_send, ec);
          });
   }
};

// --------------------------------------------------------------- StreamSession
// A WebSocket client. A read is always pending, so that Beast answers pings
// and sees the client's close; the lines are written one at a time, and the
// oldest waiting line is dropped when `client_queue` lines wait.
//
struct WsServer::Pimpl::StreamSession
    : public std::enable_shared_from_this<StreamSession>
{
   Pimpl& server;
   websocket::stream<beast::tcp_stream> ws;
   request_type req;
   beast::flat_buffer read_buffer;
   std::deque<line_ptr> queue;
   line_ptr writing = nullptr;
   bool is_open     = false; // handshake done
   bool is_closing  = false; // close once the queue is sent
   bool is_done     = false; // closed or failed: nothing more is written

   StreamSession(Pimpl& in_server, tcp::socket&& socket)
       : server(in_server)
       , ws(std::move(socket))
   {
      ++server.n_clients;
   }
   StreamSession(const StreamSession&)            = delete;
   StreamSession& operator=(const StreamSession&) = delete;
   ~StreamSession() { server.client_gone(); }

   // -------------------------------------------------------------------- start
   //
   void start(request_type&& in_req) noexcept
   {
      req = std::move(in_req);
      ws.set_option(
          websocket::stream_base::timeout::suggested(beast::role_type::server));
      ws.set_option(
          websocket::stream_base::decorator([](websocket::response_type& res) {
             res.set(http::field::server, k_server_name);
          }));
      ws.async_accept(req, [self = shared_from_this()](error_code ec) {
         self->on_accept(ec);
      });
   }

   // ---------------------------------------------------------------- on-accept
   //
   void on_accept(error_code ec) noexcept
   {
      if(ec) {
         fail(ec);
         return;
      }
      is_open = true;
      do_read();
      if(writing == nullptr) do_write();
   }

   // ------------------------------------------------------------------ do-read
   // Messages from the client are discarded
   //
   void do_read() noexcept
   {
      ws.async_read(read_buffer,
                    [self = shared_from_this()](error_code ec, size_t) {
                       if(ec) {
                          self->fail(ec);
                          return;
                       }
                       self->read_buffer.consume(self->read_buffer.size());
                       self->do_read();
                    });
   }

   // ------------------------------------------------------------------ deliver
   //
   void deliver(const line_ptr& line) noexcept
   {
      if(is_done or is_closing) return;
      if(queue.size() >= std::max<size_t>(1, server.params.client_queue)) {
         queue.pop_front();
         ++server.n_dropped;
      }
      queue.push_back(line);
      if(is_open and writing == nullptr) do_write();
   }

   // ----------------------------------------------------------------- do-write
   //
   void do_write() noexcept
   {
      if(is_done) return;
      if(queue.empty()) {
         if(is_closing) do_close();
         return;
      }
      writing = std::move(queue.front());
      queue.pop_front();
      ws.text(true);
      ws.async_write(net::buffer(*writing),
                     [self = shared_from_this()](error_code ec, size_t) {
                        self->writing = nullptr;
                        if(ec) {
                           self->fail(ec);
                           return;
                        }
                        ++self->server.n_sent;
                        self->do_write();
                     });
   }

   // -------------------------------------------------------------------- close
   // Once the queue is sent
   //
   void close() noexcept
   {
      is_closing = true;
      if(!is_open) {
         error_code ec;
         beast::get_lowest_layer(ws).socket().close(ec);
      } else if(writing == nullptr) {
         do_close();
      }
   }

   // ----------------------------------------------------------------- do-close
   //
   void do_close() noexcept
   {
      if(is_done) return;
      is_done = true;
      ws.async_close(websocket::close_code::going_away,
                     [self = shared_from_this()](error_code) {});
   }

   // --------------------------------------------------------------------- fail
   //
   void fail(error_code) noexcept
   {
      is_done = true;
      queue.clear();
   }
};

// ------------------------------------------------------------------- do-accept
//
void WsServer::Pimpl::do_accept() noexcept
{
   acceptor.async_accept([this](error_code ec, tcp::socket socket) {
      if(is_stopping) return;
      if(ec) {
         WARN(format("WebSocket server: accept failed: {}", ec.message()));
         retry_timer.expires_after(k_accept_retry);
         retry_timer.async_wait([this](error_code ec) {
            if(!ec and !is_stopping) do_accept();
         });
         return;
      }
      try {
         make_shared<HttpSession>(*this, std::move(socket))->run();
      } catch(std::exception& e) {
         LOG_ERR(format("WebSocket server: {}", e.what()));
      }
      do_accept();
   });
}

// ------------------------------------------------------------------- on-stream
//
void WsServer::Pimpl::on_stream(tcp::socket&& socket,
                                request_type&& req) noexcept
{
   if(is_stopping) return; // `socket` closes
   try {
      auto session = make_shared<StreamSession>(*this, std::move(socket));
      sessions.push_back(session);
      if(greeting != nullptr) session->deliver(greeting);
      session->start(std::move(req));
   } catch(std::exception& e) {
      LOG_ERR(format("WebSocket server: {}", e.what()));
   }
}

// --------------------------------------------------------------------- deliver
//
void WsServer::Pimpl::deliver(const line_ptr& line) noexcept
{
   auto ii = std::remove_if(begin(sessions), end(sessions), [](auto& weak) {
      return weak.expired();
   });
   sessions.erase(ii, end(sessions));
   for(auto& weak : sessions)
      if(auto session = weak.lock(); session != nullptr) session->deliver(line);
}

// ------------------------------------------------------------------- close-all
//
void WsServer::Pimpl::close_all() noexcept
{
   is_stopping = true;
   error_code ec;
   acceptor.close(ec);
   retry_timer.cancel();
   for(auto& weak : sessions)
      if(auto session = weak.lock(); session != nullptr) session->close();
}

// ----------------------------------------------------------------- client-gone
//
void WsServer::Pimpl::client_gone() noexcept
{
   {
      std::lock_guard<std::mutex> lock(padlock);
      --n_clients;
   }
   no_clients.notify_all();
}

// ----------------------------------------------------- run The server's thread
//
void WsServer::Pimpl::run() noexcept
{
   while(true) {
      try {
         ioc.run();
         return;
      } catch(std::exception& e) {
         LOG_ERR(format("WebSocket server: {}", e.what()));
      }
   }
}

// ---------------------------------------------------------------- Construction
//
WsServer::WsServer(const Params& params, health_fn health, metrics_fn metrics)
    : pimpl_(make_unique<Pimpl>())
{
   pimpl_->params  = params;
   pimpl_->health  = std::move(health);
   pimpl_->metrics = std::move(metrics);
}

WsServer::~WsServer() { stop(); }

// ------------------------------------------------------------------------ port
//
uint16_t WsServer::port() const noexcept { return pimpl_->port; }

// ------------------------------------------------------------------- n-clients
//
unsigned WsServer::n_clients() const noexcept { return pimpl_->n_clients; }

// ---------------------------------------------------------------------- n-sent
//
uint64_t WsServer::n_sent() const noexcept { return pimpl_->n_sent; }

// ------------------------------------------------------------------- n-dropped
//
uint64_t WsServer::n_dropped() const noexcept { return pimpl_->n_dropped; }

// ----------------------------------------------------------------------- start
//
bool WsServer::start() noexcept
{
   auto& P = *pimpl_;
   Expects(!P.thread.joinable());
   try {
      const tcp::endpoint endpoint{net::ip::make_address(P.params.address),
                                   P.params.port};
      P.acceptor.open(endpoint.protocol());
      P.acceptor.set_option(net::socket_base::reuse_address(true));
      P.acceptor.bind(endpoint);
      P.acceptor.listen(net::socket_base::max_listen_connections);
      P.port = P.acceptor.local_endpoint().port();
      P.work.emplace(P.ioc.get_executor());
      P.do_accept();
      P.thread     = std::thread([&P]() { P.run(); });
      P.is_running = true;
      return true;
   } catch(std::exception& e) {
      LOG_ERR(format("WebSocket server: failed to listen on {}:{}: {}",
                     P.params.address,
                     P.params.port,
                     e.what()));
   }
   return false;
}

// ------------------------------------------------------------------- broadcast
//
void WsServer::broadcast(string_view line) noexcept
{
   auto& P = *pimpl_;
   if(!P.is_running or P.n_clients == 0) return;
   try {
      auto ptr = make_shared<const string>(line);
      net::post(P.ioc, [&P, ptr = std::move(ptr)]() { P.deliver(ptr); });
   } catch(std::exception& e) {
      LOG_ERR(format("WebSocket server: {}", e.what()));
   }
}

// ---------------------------------------------------------------- set-greeting
//
void WsServer::set_greeting(string_view line) noexcept
{
   auto& P = *pimpl_;
   if(!P.is_running) return;
   try {
      auto ptr = make_shared<const string>(line);
      net::post(P.ioc, [&P, ptr = std::move(ptr)]() {
         P.greeting = ptr;
         P.deliver(ptr);
      });
   } catch(std::exception& e) {
      LOG_ERR(format("WebSocket server: {}", e.what()));
   }
}

// ------------------------------------------------------------------------ stop
//
void WsServer::stop() noexcept
{
   auto& P = *pimpl_;
   if(!P.thread.joinable()) return;
   P.is_running = false;
   try {
      net::post(P.ioc, [&P]() { P.close_all(); });
      const auto timeout
          = std::chrono::duration<double>(P.params.close_timeout_s);
      std::unique_lock<std::mutex> lock(P.padlock);
      P.no_clients.wait_for(lock, timeout, [&P]() { return P.n_clients == 0; });
   } catch(std::exception& e) {
      LOG_ERR(format("WebSocket server: {}", e.what()));
   }
   P.work.reset();
   P.ioc.stop();
   P.thread.join();
}

} // namespace perceive::pipeline::realtime
