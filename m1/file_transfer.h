/*
 * file_transfer.h
 *
 *  Created on: Apr 21, 2017
 *      Author: demo
 */

#ifndef FILE_TRANSFER_H_
#define FILE_TRANSFER_H_


int update_firmware(char * update_url,
                    uint32_t * update_hash,
                    FX_MEDIA * p_media,
                    NX_IP * p_ip,
                    NX_PACKET_POOL * p_ppool,
                    NX_DNS * p_dns,
                    const hash_instance_t * p_hash,
                    const crypto_instance_t * p_crypto,
                    char * filename,
                    const void * wdt,
                    void (* processing_callback) (int disk, int network),
                    NX_SECURE_TLS_SESSION * p_tls_session,
                    NX_SECURE_X509_CERT * p_trusted_cert
                    );


#endif /* FILE_TRANSFER_H_ */
