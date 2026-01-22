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

    //bool connect(const std::string & host, int port) { connect_impl(host, port, true, "", ""); }

    //bool connect(const std::string & host, int port, const std::string & user, const std::string & password) { connect_impl(host, port, true, "", ""); }

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
      // (se close() fallisse internamente, continuiamo comunque: qui vogliamo ripulire tutto)
      close();

      // creo la sessione SSH
      session_ = ssh_new();
      if(!session_) {
        log_err("[ssh_t] connect(): ssh_new() failed (session_ == nullptr)\n");
        return false;
      }

      // parametri di connessione
      if(ssh_options_set(session_, SSH_OPTIONS_HOST, host.c_str()) != SSH_OK) {
        log_err("[ssh_t] connect(): ssh_options_set(HOST='%s') failed: %s\n", host.c_str(), ssh_get_error(session_));
        close();
        return false;
      }

      // carico ~/.ssh/config (se presente) per Host, User, IdentityFile ecc.
      // NOTA: questa call può anche sovrascrivere opzioni precedentemente impostate in base ai match su Host.
      // Se vuoi che *sempre* vinca la porta/user passati a connect(), è OK impostarli *dopo* parse_config (come già fai).
      if(ssh_options_parse_config(session_, nullptr) != SSH_OK) {
        // Non è necessariamente fatale se non c'è config, ma libssh può ritornare errore per vari motivi.
        // Qui scegliamo di considerarlo errore per robustezza e per segnalare sempre condizioni anomale.
        log_err("[ssh_t] connect(): ssh_options_parse_config() failed: %s\n", ssh_get_error(session_));
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
        log_err("[ssh_t] connect(): ssh_connect() failed: %s\n", ssh_get_error(session_));
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
        // (nota: il thread keepalive parte più sotto, ma teniamo comunque la logica consistente).
        std::lock_guard<std::mutex> lk(io_mtx_);

        // Rimuovo prompt: riduce "rumore" nell'output (utile con read_until_done)
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
          // Non dovrebbe succedere spesso: ka_running_ true ma thread non joinable
          log_err("[ssh_t] close(): ka_running_ true but ka_thread_ not joinable\n");
        }
      }

      // Mutex: garantisce che nessun altro thread stia usando session_ o channel_ mentre li chiudiamo
      std::lock_guard<std::mutex> lk(io_mtx_);

      if(channel_) {
        // Queste funzioni spesso non ritornano errori (void), quindi logghiamo solo il flusso.
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

      // Mutex fondamentale:
      // - questo thread legge/scrive su ssh_channel
      // - il thread keepalive usa la stessa ssh_session
      // libssh NON è thread-safe -> senza mutex = race condition
      std::lock_guard<std::mutex> lk(io_mtx_);

      ++cmd_id_;
      int id = cmd_id_;

      std::string full =
      cmd + " ; echo __DONE__:" + std::to_string(id) + ":$?";

      if(!write_line(full)) {
        log_err("[ssh_t] run_command(): write_line() failed for cmd_id=%d\n", id);
        return false;
      }

      int exit_code = -1;
      if(!read_until_done(id, exit_code, out)) {
        log_err("[ssh_t] run_command(): read_until_done() failed for cmd_id=%d\n", id);
        return false;
      }

      // Se il comando ritorna != 0, la funzione ritorna false (come nel tuo design originale),
      // ma NON è un "errore di I/O": è semplicemente un exit code.
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
      std::fprintf(stderr, "%s", ""); // no-op (mantiene stile: tutte le stampe passano da qui)
      std::vfprintf(stderr, fmt, ap);
      va_end(ap);
    }

    //****************************************************************************
    // write_line()
    //
    //  - scrive una riga sulla shell
    //  - garantisce invio completo
    //****************************************************************************
    bool write_line(const std::string & s) {

      if(!connected_ || !channel_) {
        log_err("[ssh_t] write_line(): invalid state (connected_=%d, channel_=%p)\n",
                connected_ ? 1 : 0, (void*)channel_);
        return false;
      }

      std::string line = s;
      if(line.empty() || line.back() != '\n')
        line.push_back('\n');

      const char* p = line.c_str();
      int left = (int)line.size();

      while(left > 0) {
        int n = ssh_channel_write(channel_, p, left);
        if (n <= 0) {
          // ssh_channel_write ritorna <=0 in caso di errore (o 0 in alcuni casi anomali).
          log_err("[ssh_t] write_line(): ssh_channel_write() failed (n=%d): %s\n",
                  n,
                  (session_ ? ssh_get_error(session_) : "no session"));
          return false;
        }
        p += n;
        left -= n;
      }

      return true;

    }

    //****************************************************************************
    // read_until_done()
    //
    //  - legge stdout finché trova il marker del comando
    //****************************************************************************
    bool read_until_done(int id, int & exit_code, std::string * out) {

      exit_code = -1;

      std::string acc;
      char buf[4096];

      const std::string marker = "__DONE__:" + std::to_string(id) + ":";

      for(;;) {

        int n = ssh_channel_read(channel_, buf, sizeof(buf), 0);

        if(n > 0) {

          acc.append(buf, n);
          if(out) out->append(buf, n);

          size_t pos = acc.find(marker);

          if(pos != std::string::npos) {

            size_t start = pos + marker.size();
            size_t end = acc.find('\n', start);

            if(end == std::string::npos) continue;

            exit_code = std::atoi(acc.substr(start, end - start).c_str());

            return true;

          }

        } else if (n == 0) {

  #ifdef _WIN32
          Sleep(10);
  #else
          usleep(10 * 1000);
  #endif

        } else {
          // n < 0 => errore in lettura
          log_err("[ssh_t] read_until_done(): ssh_channel_read() failed (n=%d) for cmd_id=%d: %s\n",
                  n, id,
                  (session_ ? ssh_get_error(session_) : "no session"));
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

        // sleep ~20 secondi (200 * 100 ms)

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
        // (utile per tenere viva la connessione a livello TCP/NAT/firewall)
        //
        // ssh_send_ignore() ritorna SSH_OK/SSH_ERROR: logghiamo se fallisce.
        int rc = ssh_send_ignore(session_, "keepalive");
        if(rc != SSH_OK) {
          log_err("[ssh_t] keepalive_loop(): ssh_send_ignore() failed (rc=%d): %s\n",
                  rc, ssh_get_error(session_));
          // Non chiudiamo la sessione qui: è un keepalive "best effort".
          // Se vuoi che un keepalive fallito chiuda tutto, possiamo farlo.
        }

      }

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
