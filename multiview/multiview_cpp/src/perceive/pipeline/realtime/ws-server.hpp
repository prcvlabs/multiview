
#pragma once

#include "json/json.h"

namespace perceive::pipeline::realtime
{
// -------------------------------------------------------------------- WsServer
// One listener for the stream and the service's state, on one thread:
//
//    WS  /v1/stream    every line `broadcast` is given, one text message each
//    GET /v1/health    the JSON `health` returns (200)
//    GET /v1/metrics   the text `metrics` returns (Prometheus, 200)
//
// Each client has its own queue of `client_queue` lines; when a client reads
// too slowly the oldest line it has not been sent is dropped, so a client
// never holds up the stream. A client that connects late is sent the
// greeting (the stream's header) first. No TLS and no authentication.
//
class WsServer
{
 public:
   // Called on the server's thread; must not block
   using health_fn  = std::function<Json::Value()>;
   using metrics_fn = std::function<string()>;

   struct Params
   {
      string address       = "0.0.0.0"s;
      uint16_t port        = 0;   // 0: any free port, see `port()`
      size_t client_queue  = 256; // lines per client
      real close_timeout_s = 2.0; // for the clients to take their last lines
   };

 private:
   struct Pimpl;
   unique_ptr<Pimpl> pimpl_;

 public:
   WsServer(const Params& params, health_fn health, metrics_fn metrics);
   WsServer(const WsServer&) = delete;
   WsServer(WsServer&&)      = delete;
   ~WsServer(); // `stop`
   WsServer& operator=(const WsServer&) = delete;
   WsServer& operator=(WsServer&&)      = delete;

   uint16_t port() const noexcept; // the bound port, after `start`
   unsigned n_clients() const noexcept;
   uint64_t n_sent() const noexcept;    // messages, all clients
   uint64_t n_dropped() const noexcept; // lines, all clients

   // Binds and listens, and starts the thread. False on error (logged).
   bool start() noexcept;

   // Any thread: queues `line` for every connected client
   void broadcast(string_view line) noexcept;

   // Any thread: `broadcast`s `line`, and sends it first to every client that
   // connects after
   void set_greeting(string_view line) noexcept;

   // Closes every client once it has been sent its queue (or after
   // `close_timeout_s`), then joins the thread. Idempotent.
   void stop() noexcept;
};

} // namespace perceive::pipeline::realtime
