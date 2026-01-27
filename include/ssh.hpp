/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2026
 * Created by Leonardo Parisi (leonardo.parisi[at]gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _H_MPL_SSH_H_
#define _H_MPL_SSH_H_

// ============================================================
//  ssh_t.hpp
//
//   - Connessione SSH persistente
//   - Shell interattiva con PTY
//   - Esecuzione di comandi nel tempo
//   - Keepalive automatico (thread interno)
//   - run_command() -> true / false in base all'exit code
//
//  NOTA IMPORTANTE:
//   La classe usa un mutex perché esistono DUE THREAD:
//    1) thread principale (run_command)
//    2) thread keepalive automatico
//   libssh NON è thread-safe sulla stessa sessione/channel.
// ============================================================

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstdlib>
#include <cstdio>   // fprintf, stderr
#include <cstdarg>  // va_list, va_start, va_end

#include <libssh/libssh.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cerrno>
#include <cstring>


//****************************************************************************
// namespace mpl
//****************************************************************************
namespace mpl {

  //****************************************************************************
  // class ssh_t
  //****************************************************************************
  class ssh_t {

  public:

    ssh_t() : session_(nullptr), channel_(nullptr), connected_(false), cmd_id_(0), ka_running_(false) { }

    ~ssh_t() { close(); }
    
    //****************************************************************************
    // connect()
    //
    //  - apre sessione SSH
    //  - autentica con password
    //  - apre una shell persistente con PTY
    //  - avvia keepalive automatico
    //****************************************************************************
    bool connect(const std::string & host, int port, const std::string & user, const std::string & password = "") {

      // chiudo eventuale connessione precedente
      close();

      // creo la sessione SSH
      session_ = ssh_new();
      if(!session_) {
        log_err("[ssh_t] connect(): ssh_new() failed (session_ == nullptr)\n");
        return false;
      }
      
      // SSH_LOG_NOLOG     // nessun log
      // SSH_LOG_WARNING   // solo warning ed errori
      // SSH_LOG_PROTOCOL  // info sul protocollo SSH
      // SSH_LOG_PACKET    // log dei pacchetti SSH
      // SSH_LOG_FUNCTIONS // trace delle funzioni (molto verboso)
      int verb = SSH_LOG_NOLOG;
      if(ssh_options_set(session_, SSH_OPTIONS_LOG_VERBOSITY, &verb) != SSH_OK) {
        log_err("[ssh_t] connect(): ssh_options_set(VERBOSE='%s') failed: %s\n", host.c_str(), ssh_get_error(session_));
        close();
        return false;
      }

      // imposto l'host
      if(ssh_options_set(session_, SSH_OPTIONS_HOST, host.c_str()) != SSH_OK) {
        log_err("[ssh_t] connect(): ssh_options_set(HOST='%s') failed: %s\n", host.c_str(), ssh_get_error(session_));
        close();
        return false;
      }

      // imposto la porta
      if(ssh_options_set(session_, SSH_OPTIONS_PORT, &port) != SSH_OK) {
        log_err("[ssh_t] connect(): ssh_options_set(PORT=%d) failed: %s\n", port, ssh_get_error(session_));
        close();
        return false;
      }

      // imposto l'user
      if(ssh_options_set(session_, SSH_OPTIONS_USER, user.c_str()) != SSH_OK) {
        log_err("[ssh_t] connect(): ssh_options_set(USER='%s') failed: %s\n", user.c_str(), ssh_get_error(session_));
        close();
        return false;
      }

      // timeout di rete (secondi)
      int timeout_sec = 10;
      if(ssh_options_set(session_, SSH_OPTIONS_TIMEOUT, &timeout_sec) != SSH_OK) {
        log_err("[ssh_t] connect(): ssh_options_set(TIMEOUT=%d) failed: %s\n", timeout_sec, ssh_get_error(session_));
        close();
        return false;
      }

      // handshake TCP + SSH
      if(ssh_connect(session_) != SSH_OK) {
        int ecode = ssh_get_error_code(session_);
        log_err("[ssh_t] connect(): ssh_connect() failed (ecode=%d): %s\n", ecode, ssh_get_error(session_));
        close();
        return false;
      }

      // stato dell'autenticazione
      int rc = SSH_AUTH_ERROR;

      // authorized_keys
      if(password.empty()) {

        rc = ssh_userauth_publickey_auto(session_, nullptr, nullptr);
        if(rc != SSH_AUTH_SUCCESS) {
          log_err("[ssh_t] connect(): ssh_userauth_publickey_auto() failed (rc=%d): %s\n", rc, ssh_get_error(session_));
          close();
          return false;
        }

      // user + password
      } else {

        rc = ssh_userauth_password(session_, nullptr, password.c_str());
        if(rc != SSH_AUTH_SUCCESS) {
          log_err("[ssh_t] connect(): ssh_userauth_password() failed (rc=%d): %s\n", rc, ssh_get_error(session_));
          close();
          return false;
        }

      }

      // creo un canale SSH
      channel_ = ssh_channel_new(session_);
      if(!channel_) {
        log_err("[ssh_t] connect(): ssh_channel_new() failed (channel_ == nullptr): %s\n", ssh_get_error(session_));
        close();
        return false;
      }

      // apro sessione sul canale
      if(ssh_channel_open_session(channel_) != SSH_OK) {
        log_err("[ssh_t] connect(): ssh_channel_open_session() failed: %s\n", ssh_get_error(session_));
        close();
        return false;
      }

      // richiedo un PTY (necessario per shell interattiva / conda)
      if(ssh_channel_request_pty(channel_) != SSH_OK) {
        log_err("[ssh_t] connect(): ssh_channel_request_pty() failed: %s\n", ssh_get_error(session_));
        close();
        return false;
      }

      // apro shell persistente
      if(ssh_channel_request_shell(channel_) != SSH_OK) {
        log_err("[ssh_t] connect(): ssh_channel_request_shell() failed: %s\n", ssh_get_error(session_));
        close();
        return false;
      }

      // ============================================================
      // FIX BUG ORIGINALE:
      //  Prima tu chiamavi write_line() con connected_ == false,
      //  e write_line() faceva return false "sempre".
      //
      //  Ora impostiamo connected_ = true subito dopo aver aperto
      //  correttamente la shell (channel + PTY + shell OK).
      // ============================================================
      connected_ = true;

      bool writeIsOk = true;
      
      {
        // Mutex necessario: questa scrittura usa ssh_channel e potrebbe sovrapporsi al thread keepalive
        std::lock_guard<std::mutex> lk(io_mtx_);

        // Disabilita bracketed paste in bash/readline
        if(!write_line("bind 'set enable-bracketed-paste off' 2>/dev/null || true")) writeIsOk = false;

        // Rimuovi eventuale PROMPT_COMMAND che può riscrivere cose sul terminale
        if(!write_line("unset PROMPT_COMMAND 2>/dev/null || true")) writeIsOk = false;
        
        // disabilita conda prompt modifier
        if(!write_line("export CONDA_CHANGEPS1=false")) writeIsOk = false;
        
        // Rimuovo prompt della shell
        if(!write_line("export PS1=''")) writeIsOk = false;

        // Disabilito echo dei comandi: evita che i comandi inviati compaiano in output
        if(!write_line("stty -echo"))    writeIsOk = false;
        
      }
      
      if(!writeIsOk) {
        log_err("[ssh_t] connect(): initial write_line() failed (PS1 / stty)\n");
        close();
        return false;
      }

      // avvio thread keepalive automatico
      ka_running_ = true;
      ka_thread_ = std::thread(&ssh_t::keepalive_loop, this);

      return true;

    }

    //****************************************************************************
    // close()
    //
    //  - ferma keepalive
    //  - chiude canale e sessione SSH
    //****************************************************************************
    void close() {

      // fermo thread keepalive
      if(ka_running_) {
        ka_running_ = false;
        if(ka_thread_.joinable()) {
          ka_thread_.join();
        } else {
          log_err("[ssh_t] close(): ka_running_ true but ka_thread_ not joinable\n");
        }
      }

      // Mutex: garantisce che nessun altro thread stia usando session_ o channel_ mentre li chiudiamo
      std::lock_guard<std::mutex> lk(io_mtx_);

      if(channel_) {
        ssh_channel_send_eof(channel_);
        ssh_channel_close(channel_);
        ssh_channel_free(channel_);
        channel_ = nullptr;
      }

      if(session_) {
        ssh_disconnect(session_);
        ssh_free(session_);
        session_ = nullptr;
      }

      connected_ = false;

    }

    //****************************************************************************
    // run_command()
    //
    //  - esegue un comando nella shell persistente
    //  - attende il marker "__DONE__:id:$?"
    //  - ritorna true se exit code == 0
    //****************************************************************************
    bool run_command(const std::string & cmd, std::string * out = nullptr) {

      if(!connected_) {
        log_err("[ssh_t] run_command(): not connected\n");
        return false;
      }

      // Mutex fondamentale
      // questo thread legge/scrive su ssh_channel.
      // il thread keepalive usa la stessa ssh_session
      // libssh NON è thread-safe -> senza mutex = race condition
      std::lock_guard<std::mutex> lk(io_mtx_);

      ++cmd_id_;
      int id = cmd_id_;

      std::string full = "echo __BEGIN__:" + std::to_string(id) + " ; " + cmd + " ; echo __DONE__:" + std::to_string(id) + ":$?";

      if(!write_line(full)) {
        log_err("[ssh_t] run_command(): write_line() failed for cmd_id=%d\n", id);
        return false;
      }

      int exit_code = -1;
      if(!read_until_done(id, exit_code, out)) {
        log_err("[ssh_t] run_command(): read_until_done() failed for cmd_id=%d\n", id);
        return false;
      }

      // se il comando ritorna != 0, la funzione ritorna false
      return (exit_code == 0);

    }

  private:

    //****************************************************************************
    // log_err()
    //
    //  - helper per scrivere su stderr in modo uniforme
    //  - evita ripetizioni di fprintf(stderr, ...) in ogni punto
    //****************************************************************************
    static void log_err(const char* fmt, ...) {
      va_list ap;
      va_start(ap, fmt);
      std::fprintf(stderr, "%s", "");
      std::vfprintf(stderr, fmt, ap);
      va_end(ap);
    }

    //****************************************************************************
    // write_line()
    //
    //  - scrive una riga sulla shell
    //  - garantisce invio completo
    //****************************************************************************
    bool write_line(std::string line) {

      if(!connected_ || !channel_) {
        log_err("[ssh_t] write_line(): invalid state (connected_=%d, channel_=%p)\n", connected_ ? 1 : 0, (void*)channel_);
        return false;
      }

      if(line.empty() || line.back() != '\n')
        line.push_back('\n');

      const char * p = line.c_str();
      int left = (int)line.size();

      while(left > 0) {
        int n = ssh_channel_write(channel_, p, left);
        if(n <= 0) {
          log_err("[ssh_t] write_line(): ssh_channel_write() failed (n=%d): %s\n", n, (session_ ? ssh_get_error(session_) : "no session"));
          return false;
        }
        p += n;
        left -= n;
      }

      return true;

    }

    static constexpr std::size_t SSH_READ_BUF_SIZE = 4096;

    //****************************************************************************
    // read_until_done()
    //
    //  - legge stdout finché trova il marker del comando
    //****************************************************************************
    bool read_until_done(int id, int & exit_code, std::string * out) {

      exit_code = -1;

      std::string acc;
      char buf[SSH_READ_BUF_SIZE];

      const std::string begin_marker = "__BEGIN__:" + std::to_string(id);
      const std::string done_marker  = "__DONE__:"  + std::to_string(id) + ":";

      // indici nella stringa accumulata
      std::size_t begin_line_end = std::string::npos; // fine linea dopo BEGIN
      std::size_t done_pos       = std::string::npos; // posizione del marker DONE

      for(;;) {

        int n = ssh_channel_read(channel_, buf, sizeof(buf), 0);

        if(n > 0) {

          acc.append(buf, n);

          // 1) Trova BEGIN (una sola volta) e la fine riga di BEGIN
          if(begin_line_end == std::string::npos) {

            std::size_t bpos = acc.find(begin_marker);
            
            if(bpos != std::string::npos) {

              // la riga di BEGIN deve finire con '\n'
              std::size_t nl = acc.find('\n', bpos);
              if(nl != std::string::npos) {
                begin_line_end = nl + 1; // output "vero" parte da qui
              }
              
            }
            
          }

          // 2) Trova DONE (solo dopo aver visto BEGIN completo)
          if(begin_line_end != std::string::npos) {

            done_pos = acc.find(done_marker, begin_line_end);
            
            if(done_pos != std::string::npos) {

              // parse exit code
              std::size_t code_start = done_pos + done_marker.size();
              std::size_t code_end   = acc.find('\n', code_start);

              if(code_end == std::string::npos) {
                // marker trovato ma exit code non completo
                continue;
              }

              // out = solo tra fine riga BEGIN e inizio DONE
              if(out) {
                out->assign(acc.data() + begin_line_end, done_pos - begin_line_end);
              }

              exit_code = std::atoi(acc.substr(code_start, code_end - code_start).c_str());
              
              return true;
            }
            
            
          }

        } else if(n == 0) {

    #ifdef _WIN32
          Sleep(10);
    #else
          usleep(10 * 1000);
    #endif

        if(ssh_channel_is_eof(channel_) || ssh_channel_is_closed(channel_)) {
            log_err("[ssh_t] read_until_done(): channel closed before marker for cmd_id=%d\n", id);
            return false;
          }

        } else {
          log_err("[ssh_t] read_until_done(): ssh_channel_read() failed (n=%d) for cmd_id=%d: %s\n", n, id, (session_ ? ssh_get_error(session_) : "no session"));
          return false;
        }
        
      }
      
    }

    //****************************************************************************
    // keepalive_loop()
    //
    //  - thread interno
    //  - invia keepalive SSH ogni ~20 secondi
    //****************************************************************************
    void keepalive_loop() {

      while (ka_running_) {

        for(int i = 0; i < 200; ++i) {

          if(!ka_running_) return;
  #ifdef _WIN32
          Sleep(100);
  #else
          usleep(100 * 1000);
  #endif
        }

        if(!connected_ || !session_) continue;

        // Mutex necessario:
        // evita che il keepalive interferisca con
        // ssh_channel_read / ssh_channel_write in run_command()
        std::lock_guard<std::mutex> lk(io_mtx_);

        // Keepalive robusto e sempre disponibile: manda un SSH_MSG_IGNORE
        int rc = ssh_send_ignore(session_, "keepalive");
        if(rc != SSH_OK) {
          log_err("[ssh_t] keepalive_loop(): ssh_send_ignore() failed (rc=%d): %s\n", rc, ssh_get_error(session_));
          ka_running_ = false;
          invalidate_connection();
          return;
        }

      }

    }

    //****************************************************************************
    // invalidate_connection()
    //****************************************************************************
    void invalidate_connection() {

      std::lock_guard<std::mutex> lk(io_mtx_);

      if(channel_) {
        ssh_channel_send_eof(channel_);
        ssh_channel_close(channel_);
        ssh_channel_free(channel_);
        channel_ = nullptr;
      }

      if(session_) {
        ssh_disconnect(session_);
        ssh_free(session_);
        session_ = nullptr;
      }

      connected_ = false;
      
    }

    
  private:

    ssh_session session_;          // sessione SSH
    ssh_channel channel_;          // shell persistente
    bool connected_;               // stato connessione
    int cmd_id_;                   // id progressivo comandi

    std::thread ka_thread_;        // thread keepalive
    std::atomic<bool> ka_running_; // flag thread
    std::mutex io_mtx_;            // protegge session_ e channel_

  };

} // namespace


#endif
