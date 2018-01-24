/*
 *  Copyright (c) 2017 Medium One, Inc
 *  www.mediumone.com
 *
 *  Portions of this work may be based on third party contributions.
 *  Medium One, Inc reserves copyrights to this work whose
 *  license terms are defined under a separate Software License
 *  Agreement (SLA).  Re-distribution of any or all of this work,
 *  in source or binary form, is prohibited unless authorized by
 *  Medium One, Inc under SLA.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef M1_H_
#define M1_H_


#include <nx_api.h>
#include <nxd_dns.h>
#include <nxd_mqtt_client.h>


#define M1_VERSION_STRING "1.3.2"
#define MAX_CERT_LENGTH 1536
#define MAX_KEY_LENGTH 2048


extern const unsigned char M1_ROOT_CERT[1082];

// Error Codes
enum {
    M1_SUCCESS = 0,
    M1_ERROR_INVALID_URL,
    M1_ERROR_UNABLE_TO_CONNECT,
    M1_ERROR_NOT_CONNECTED,
    M1_ERROR_ALREADY_CONNECTED,
    M1_ERROR_NULL_PAYLOAD,
    M1_ERROR_UNABLE_TO_PUBLISH,
    M1_ERROR_NULL_CALLBACK,
    M1_ERROR_BAD_CREDENTIALS,
};


typedef struct {
    char user_id[12];
    char password[64];
} user_credentials_t;


typedef struct {
    char apikey[49];
    char proj_id[12];
} project_credentials_t;


typedef struct {
    const char * mqtt_url;
    int mqtt_port;
    const project_credentials_t * project;
    const user_credentials_t * registration;
    user_credentials_t * device;
    const char * device_id;
    int retry_limit;
    int retry_delay;
    int mqtt_heart_beat;
    int tls_enabled;
    void * stack_mem;
    int stack_mem_size; // min?
    void * mqtt_msg_mem;
    int mqtt_msg_mem_size; // 2k * 10?
    void * ssl_mem;
    int ssl_mem_size; // min? must be able to accomodate max mqtt msg size + TLS overhead
    void * crypto_mem;
    int crypto_mem_size; // 9k for s7 crypto table
    void * cert_mem;
    int cert_mem_size; // 2k * 3 (3 certs in m1 chain)
    NX_PACKET_POOL * p_ppool;
    NX_IP * p_ip;
    NX_DNS * p_dns;
    const char * login_id;
    const char * device_cert;
    const char * device_key;
    NXD_MQTT_CLIENT * p_mqtt_client;
} m1_connect_params;


#endif /* M1_H_ */
