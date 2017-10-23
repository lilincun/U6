/**
****************************************************************************************
*
* @file cmd_handlers.c
*
* @brief Handling of CLI commands provided on command line
*
* Copyright (C) 2015. Dialog Semiconductor, unpublished work. This computer
* program includes Confidential, Proprietary Information and is a Trade Secret of
* Dialog Semiconductor. All use, disclosure, and/or reproduction is prohibited
* unless authorized in writing. All Rights Reserved.
*
* <black.orca.support@diasemi.com> and contributors.
*
****************************************************************************************
*/
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <programmer.h>
#include "cli_common.h"
#include <assert.h>

#define DEFAULT_SIZE 0x100000

/* Maximum size for an image */
#define MAX_IMAGE_SIZE 0x7F000

static int cmdh_write(int argc, char *argv[]);
static int cmdh_read(int argc, char *argv[]);
static int cmdh_write_qspi(int argc, char *argv[]);
static int cmdh_write_qspi_bytes(int argc, char *argv[]);
static int cmdh_write_qspi_exec(int argc, char *argv[]);
static int cmdh_write_suota_image(int argc, char *argv[]);
static int cmdh_read_qspi(int argc, char *argv[]);
static int cmdh_read_partition_table(int argc, char *argv[]);
static int cmdh_erase_qspi(int argc, char *argv[]);
static int cmdh_chip_erase_qspi(int argc, char *argv[]);
static int cmdh_copy_qspi(int argc, char *argv[]);
static int cmdh_is_empty_qspi(int argc, char *argv[]);
static int cmdh_write_otp(int argc, char *argv[]);
static int cmdh_read_otp(int argc, char *argv[]);
static int cmdh_write_otp_file(int argc, char *argv[]);
static int cmdh_write_otp_raw_file(int argc, char *argv[]);
static int cmdh_read_otp_file(int argc, char *argv[]);
static int cmdh_write_tcs(int argc, char *argv[]);
static int cmdh_boot(int argc, char *argvp[]);
static int cmdh_read_chip_info(int argc, char *argvp[]);


/**
 * \brief CLI command handler description
 *
 */
struct cli_command {
        const char *name;                       /**< name of command */
        int min_num_p;                          /**< minimum number of parameters */
        int (* func) (int argc, char *argv[]);  /**< handler function, return non-zero for success */
};

/**
 * \brief CLI command handlers
 *
 */
static struct cli_command cmds[] = {
        { "write",                2, cmdh_write, },
        { "read",                 3, cmdh_read, },
        { "write_qspi",           2, cmdh_write_qspi, },
        { "write_qspi_bytes",     2, cmdh_write_qspi_bytes, },
        { "write_qspi_exec",      1, cmdh_write_qspi_exec, },
        { "write_suota_image",    2, cmdh_write_suota_image, },
        { "read_qspi",            3, cmdh_read_qspi, },
        { "erase_qspi",           2, cmdh_erase_qspi, },
        { "chip_erase_qspi",      0, cmdh_chip_erase_qspi, },
        { "read_partition_table", 0, cmdh_read_partition_table, },
        { "copy_qspi",            3, cmdh_copy_qspi, },
        { "is_empty_qspi",        0, cmdh_is_empty_qspi, },
        { "write_otp",            2, cmdh_write_otp, },
        { "read_otp",             2, cmdh_read_otp, },
        { "write_otp_file",       1, cmdh_write_otp_file, },
        { "write_otp_raw_file",   2, cmdh_write_otp_raw_file, },
        { "read_otp_file",        1, cmdh_read_otp_file, },
        { "write_tcs",            3, cmdh_write_tcs },
        { "boot",                 0, cmdh_boot, },
        { "read_chip_info",       0, cmdh_read_chip_info, },
        /* end of table */
        { NULL,             0, NULL, }
};

static int get_filesize(const char *fname)
{
        struct stat st;

        if (stat(fname, &st) < 0) {
                return -1;
        }

        return st.st_size;
}

static bool check_otp_cell_address(uint32_t *addr)
{
        uint32_t _addr = *addr;

        /* convert mapped address to cell address, if possible */
        if ((_addr & 0x07F80000) == 0x07F80000) {
                _addr = (_addr & 0xFFFF) >> 3;
        }

        /* there are 0x2000 cells... */
        if (_addr >= 0x2000) {
                return false;
        }

        *addr = _addr;

        return true;
}

static int cmdh_write(int argc, char *argv[])
{
        unsigned int addr;
        const char *fname = argv[1];
        unsigned int size = 0;
        int ret;

        if (!get_number(argv[0], &addr)) {
                fprintf(stderr, "invalid address\n");
                return 0;
        }

        if (argc > 2) {
                if (!get_number(argv[2], &size)) {
                        fprintf(stderr, "invalid size\n");
                        return 0;
                }
        } else {
                int file_size = get_filesize(fname);

                if (file_size < 0) {
                        fprintf(stderr, "could not open file\n");
                        return 0;
                }

                size = file_size;
        }

        ret = prog_write_file_to_ram(addr, fname, size);
        if (ret) {
                fprintf(stderr, "write to RAM failed: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }

        return 1;
}

static int cmdh_read(int argc, char *argv[])
{
        unsigned int addr;
        const char *fname = argv[1];
        unsigned int size = 0;
        uint8_t *buf = NULL;
        int ret;

        if (!get_number(argv[0], &addr)) {
                fprintf(stderr, "invalid address\n");
                return 0;
        }

        if (!get_number(argv[2], &size)) {
                fprintf(stderr, "invalid size\n");
                return 0;
        }

        if (!strcmp(fname, "-") || !strcmp(fname, "--")) {
                buf = malloc(size);
                ret = prog_read_memory(addr, buf, size);
        } else {
                ret = prog_read_memory_to_file(addr, fname, size);
        }
        if (ret) {
                free(buf);
                fprintf(stderr, "read from RAM failed: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }

        if (buf) {
                dump_hex(addr, buf, size, !strcmp(fname, "--") ? 32 : 16);
                free(buf);
        }

        return 1;
}

static int cmdh_write_qspi(int argc, char *argv[])
{
        unsigned int addr;
        const char *fname = argv[1];
        unsigned int size = 0;
        int ret;

        if (!get_number(argv[0], &addr)) {
                fprintf(stderr, "invalid address\n");
                return 0;
        }

        if (argc > 2) {
                if (!get_number(argv[2], &size)) {
                        fprintf(stderr, "invalid size\n");
                        return 0;
                }
        } else {
                int file_size = get_filesize(fname);

                if (file_size < 0) {
                        fprintf(stderr, "could not open file\n");
                        return 0;
                }

                size = file_size;
        }

        ret = prog_write_file_to_qspi(addr, fname, size);
        if (ret) {
                fprintf(stderr, "write to QSPI failed: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }

        return 1;
}

static int cmdh_write_qspi_bytes(int argc, char *argv[])
{
        unsigned int addr;
        int ret;
        int i;
        unsigned int b;
        uint8_t *buf = (uint8_t *) malloc(argc);

        if (buf == NULL) {
                return 0;
        }

        if (!get_number(argv[0], &addr)) {
                fprintf(stderr, "invalid address %s\n", argv[0]);
                ret = 0;
                goto end;
        }

        for (i = 1; i < argc; ++i) {
                if (!get_number(argv[i], &b)) {
                        fprintf(stderr, "invalid byte '%s'\n", argv[i]);
                        ret = 0;
                        goto end;
                }
                buf[i - 1] = (uint8_t) b;
        }

        ret = prog_write_to_qspi(addr, buf, argc - 1);
        if (ret) {
                fprintf(stderr, "write to QSPI failed: %s (%d)\n", prog_get_err_message(ret), ret);
                ret = 0;
                goto end;
        }
end:
        if (buf) {
                free(buf);
        }

        return 1;
}

/*
 *  Write image to address 0, qQ are added after everything else.
 */
static int prog_write_image_to_qspi_safe(uint8_t *buf, uint32_t size)
{
        int err;
        uint8_t head[2] = { buf[0], buf[1] };

        buf[0] = 0xFF;
        buf[1] = 0xFF;

        err = prog_write_to_qspi(0, buf, size);

        if (!err) {
                err = prog_write_to_qspi(0, head, 2);
        }

        return err;
}

static int prog_write_qspi_exec(uint8_t *buf, int size)
{
        int err;
        chip_info_t chip_info;
        uint8_t *buf_with_hdr;
        int size_with_hdr = size + IMAGE_HEADER_SIZE;

        if (buf == NULL) {
                return ERR_PROG_INVALID_ARGUMENT;
        }

        err = prog_read_chip_info(&chip_info);
        if (err) {
                return err;
        }

        /* Chip revision specified, compare with what read from board */
        if (main_opts.chip_rev && *main_opts.chip_rev) {
                if (strcmp(chip_info.chip_rev, main_opts.chip_rev)) {
                        return ERR_PROG_QSPI_IMAGE_FORMAT;
                }
        }

        /* Buffer must be big enough to contain binary + header */
        buf_with_hdr = (uint8_t *) malloc(size_with_hdr);

        if (!buf_with_hdr) {
                return ERR_PROG_INSUFICIENT_BUFFER;
        }

        err = prog_make_image(buf, size, chip_info.chip_rev, IMG_QSPI, IMG_CACHED, buf_with_hdr,
                                                                        size_with_hdr, NULL);

        if (err >= 0) {
                err = prog_write_image_to_qspi_safe(buf_with_hdr, size_with_hdr);
        }

        free(buf_with_hdr);

        return err;
}

int prog_write_qspi_executable_file(const char *file_name)
{
        int err = 0;
        uint8_t *buf = NULL;
        FILE *f = NULL;
        struct stat fstat;
        int size;

        if (stat(file_name, &fstat) < 0 || !S_ISREG(fstat.st_mode)) {
                return ERR_FILE_OPEN;
        }

        size = (int) fstat.st_size;
        f = fopen(file_name, "rb");
        if (f == NULL) {
                err = ERR_FILE_OPEN;
                goto end;
        }

        buf = (uint8_t *) malloc(size);
        if (buf == NULL) {
                err = ERR_ALLOC_FAILED;
                goto end;
        }

        if (fread(buf, 1, size, f) != size) {
                err = ERR_FILE_READ;
                goto end;
        }

        err = prog_write_qspi_exec(buf, fstat.st_size);
end:
        if (buf) {
                free(buf);
        }

        return err;
}

static int cmdh_write_qspi_exec(int argc, char *argv[])
{
        const char *fname = argv[0];
        struct stat fstat;
        int ret;

        if (stat(fname, &fstat) < 0 || !S_ISREG(fstat.st_mode)) {
                fprintf(stderr, "Invalid file specified %s\n", fname);
                return 0;
        }

        if (fstat.st_size > MAX_IMAGE_SIZE) {
                fprintf(stderr, "File %s is too big\n", fname);
                return 0;
        }

        ret = prog_write_qspi_executable_file(fname);
        if (ret) {
                fprintf(stderr, "Write executable failed: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }
        return 1;
}

int prog_write_qspi_suota_image_file(const char *file_name, const char *version)
{
        int err = 0;
        uint8_t *buf = NULL;
        FILE *f = NULL;
        struct stat fstat;
        int size;

        if (stat(file_name, &fstat) < 0 || !S_ISREG(fstat.st_mode)) {
                return ERR_FILE_OPEN;
        }

        size = (int) fstat.st_size;
        f = fopen(file_name, "rb");
        if (f == NULL) {
                err = ERR_FILE_OPEN;
                goto end;
        }

        buf = (uint8_t *) malloc(size);
        if (buf == NULL) {
                err = ERR_ALLOC_FAILED;
                goto end;
        }

        if (fread(buf, 1, size, f) != size) {
                err = ERR_FILE_READ;
                goto end;
        }

        err = prog_write_qspi_suota_image(buf, fstat.st_size, version, fstat.st_mtime, 0xFFFF);
end:
        if (buf) {
                free(buf);
        }

        return err;
}

static int cmdh_write_suota_image(int argc, char *argv[])
{
        const char *fname = argv[0];
        const char *version = argv[1];
        struct stat fstat;
        int ret;

        if (stat(fname, &fstat) < 0 || !S_ISREG(fstat.st_mode)) {
                fprintf(stderr, "Invalid file specified %s\n", fname);
                return 0;
        }

        if (fstat.st_size > MAX_IMAGE_SIZE) {
                fprintf(stderr, "File %s is too big\n", fname);
                return 0;
        }

        ret = prog_write_qspi_suota_image_file(fname, version);
        if (ret) {
                fprintf(stderr, "Write SUOTA image failed: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }
        return 1;
}

static int cmdh_read_qspi(int argc, char *argv[])
{
        unsigned int addr;
        const char *fname = argv[1];
        unsigned int size = 0;
        uint8_t *buf = NULL;
        int ret;

        if (!get_number(argv[0], &addr)) {
                fprintf(stderr, "invalid address\n");
                return 0;
        }

        if (!get_number(argv[2], &size)) {
                fprintf(stderr, "invalid size\n");
                return 0;
        }

        if (!strcmp(fname, "-") || !strcmp(fname, "--")) {
                buf = malloc(size);
                if (!buf) {
                        ret = ERR_ALLOC_FAILED;
                } else {
                        ret = prog_read_qspi(addr, buf, size);
                }
        } else {
                ret = prog_read_qspi_to_file(addr, fname, size);
        }
        if (ret) {
                free(buf);
                fprintf(stderr, "read from QSPI failed: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }

        if (buf) {
                dump_hex(addr, buf, size, !strcmp(fname, "--") ? 32 : 16);
                free(buf);
        }

        return 1;
}

static int cmdh_erase_qspi(int argc, char *argv[])
{
        unsigned int addr;
        unsigned int size;
        int ret;

        if (!get_number(argv[0], &addr)) {
                fprintf(stderr, "invalid address\n");
                return 0;
        }

        if (!get_number(argv[1], &size)) {
                fprintf(stderr, "invalid size\n");
                return 0;
        }

        ret = prog_erase_qspi(addr, size);
        if (ret) {
                fprintf(stderr, "erase QSPI failed: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }

        return 1;
}

static int cmdh_read_partition_table(int argc, char *argv[])
{
        int ret = 0;
        unsigned int size = 0;
        uint8_t *buf = NULL;

        ret = prog_read_partition_table(&buf, &size);
        if (ret) {
                fprintf(stderr, "read partition table failed: %s (%d)\n", prog_get_err_message(ret), ret);
                goto done;
        }

        ret = dump_partition_table(buf, size);

done:
        free(buf);
        return (ret == 0);
}

static int cmdh_copy_qspi(int argc, char *argv[])
{
        unsigned int addr_ram;
        unsigned int addr_qspi;
        unsigned int size;
        int ret;

        if (!get_number(argv[0], &addr_ram)) {
                fprintf(stderr, "invalid RAM address\n");
                return 0;
        }

        if (!get_number(argv[1], &addr_qspi)) {
                fprintf(stderr, "invalid QSPI address\n");
                return 0;
        }

        if (!get_number(argv[2], &size)) {
                fprintf(stderr, "invalid size\n");
                return 0;
        }

        ret = prog_copy_to_qspi(addr_ram, addr_qspi, size);
        if (ret) {
                fprintf(stderr, "erase QSPI failed: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }

        return 1;
}

static int cmdh_is_empty_qspi(int argc, char *argv[])
{
        unsigned int size = DEFAULT_SIZE;
        unsigned int start_address = 0;
        int ret_number;
        int ret = 0;

        if (argc != 0 && argc != 2) {
                fprintf(stderr, "invalid argument - function is_empty_qspi needs zero or two arguments\n");
                return -1;
        }

        if (argc == 2) {
                if (!get_number(argv[0], &start_address)) {
                        fprintf(stderr, "invalid start address\n");
                        return -1;
                }

                if (!get_number(argv[1], &size) || size == 0) {
                        fprintf(stderr, "invalid size\n");
                        return -1;
                }
        }

        ret = prog_is_empty_qspi(size, start_address, &ret_number);

        if (!ret) {
                if (ret_number <= 0) {
                        printf("QSPI flash region is not empty (byte at 0x%08x + 0x%08x is not 0xFF).\n",
                                                                start_address, (-1 * ret_number));
                } else {
                        printf("QSPI flash region is empty (checked %u bytes).\n", ret_number);
                }
        } else {
                fprintf(stderr, "check QSPI emptiness failed: %s (%d)\n", prog_get_err_message(ret),
                                                                                        ret);
        }

        return ret;
}

static int cmdh_chip_erase_qspi(int argc, char *argv[])
{
        int ret;

        ret = prog_chip_erase_qspi();

        if (ret) {
                fprintf(stderr, "chip erase QSPI failed: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }

        return 1;
}

static int cmdh_write_otp(int argc, char *argv[])
{
        unsigned int addr;
        unsigned int length;
        uint32_t *buf;
        int i;
        int ret;

        if (!get_number(argv[0], &addr) || !check_otp_cell_address(&addr)) {
                fprintf(stderr, "invalid address\n");
                return 0;
        }

        if (!get_number(argv[1], &length) || !length) {
                fprintf(stderr, "invalid length\n");
                return 0;
        }

        argc -= 2;
        argv += 2;

        buf = calloc(length, sizeof(*buf));
        for (i = 0; i < argc; i++) {
                if (!get_number(argv[i], &buf[i])) {
                        fprintf(stderr, "invalid data (#%d)\n", i + 1);
                        ret = 0;
                        goto done;
                }
        }

        ret = prog_write_otp(addr, buf, length);
        if (ret) {
                fprintf(stderr, "write to OTP failed: %s (%d)\n", prog_get_err_message(ret), ret);
                ret = 0;
                goto done;
        }

        ret = 1;

done:
        free(buf);

        return ret;
}

static int cmdh_read_otp(int argc, char *argv[])
{
        unsigned int addr;
        unsigned int length;
        unsigned int size;
        uint32_t *buf;
        int ret;

        if (!get_number(argv[0], &addr) || !check_otp_cell_address(&addr)) {
                fprintf(stderr, "invalid address\n");
                return 0;
        }

        if (!get_number(argv[1], &length) || !length) {
                fprintf(stderr, "invalid length\n");
                return 0;
        }
		
        size = length * sizeof(*buf);

        buf = malloc(size);
		assert(buf != NULL);

        ret = prog_read_otp(addr, buf, length);
        if (ret) {
                fprintf(stderr, "read from OTP failed: %s (%d)\n", prog_get_err_message(ret), ret);
                ret = 0;
                goto done;
        }

        dump_otp(addr, buf, length);

        ret = 1;

done:
        free(buf);

        return ret;
}
static bool write_otp_file_value_cb(uint32_t addr, uint32_t size, uint64_t value)
{
        uint8_t *buf;
        unsigned int i;
        int ret;

        buf = calloc(1, size);
        if (!buf) {
                return false;
        }

        memcpy(buf, &value, size < sizeof(value) ? size : sizeof(value));

        printf("write_otp %04x %d ", addr, size);
        for (i = 0; i < size; i++) {
                printf("%02X", buf[i]);
        }

        ret = prog_write_otp(addr, (void *) buf, size / 4);
        if (ret < 0) {
                printf(" (FAILED: %s (%d))\n", prog_get_err_message(ret), ret);
                free(buf);
                return false;
        }

        free(buf);
        printf(" (OK)\n");

        return true;
}

int cmdh_write_otp_file(int argc, char *argv[])
{
        return parse_otp_file(argv[0], write_otp_file_value_cb);
}

static int cmdh_write_otp_raw_file(int argc, char *argv[])
{
        unsigned int addr;
        const char *fname = argv[1];
        unsigned int size = 0;
        int ret;

        if (!get_number(argv[0], &addr) || !check_otp_cell_address(&addr)) {
                fprintf(stderr, "invalid address\n");
                return 0;
        }

        if (argc > 2) {
                if (!get_number(argv[2], &size)) {
                        fprintf(stderr, "invalid size\n");
                        return 0;
                }
        } else {
                int file_size = get_filesize(fname);

                if (file_size < 0) {
                        fprintf(stderr, "could not open file\n");
                        return 0;
                }

                size = file_size;
        }

        ret = prog_write_file_to_otp(addr, fname, size);
        if (ret) {
                fprintf(stderr, "write to OTP failed: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }

        return 1;
}

static bool read_otp_file_value_cb(uint32_t addr, uint32_t size, uint64_t value)
{
        uint8_t *buf;
        int ret;

        buf = calloc(1, size);
        if (!buf) {
                return false;
        }

        printf("read_otp %04x %d ", addr, size);

        ret = prog_read_otp(addr, (void *) buf, size / 4);
        if (ret < 0) {
                printf(" (FAILED: %s (%d))\n", prog_get_err_message(ret), ret);
                free(buf);
                return false;
        }

        printf(" (OK)\n");

        dump_otp(addr, (void *) buf, size / 4);

        free(buf);

        return true;
}

int cmdh_read_otp_file(int argc, char *argv[])
{
        return parse_otp_file(argv[0], read_otp_file_value_cb);
}

static int cmdh_write_tcs(int argc, char *argv[])
{
        unsigned int length;
        uint32_t *buf;
        unsigned int i;
        uint32_t address;
        int ret;

        if (!get_number(argv[0], &length) || !length) {
                fprintf(stderr, "invalid length\n");
                return 0;
        }

        argc -= 1;
        argv += 1;

        if(length & 0x01){
                fprintf(stderr, "invalid length. TCS entries need to be in pairs\n");
                return 0;
        }
        if(length > TCS_WORD_SIZE){
                fprintf(stderr, "invalid length. length is bigger than TCS size\n");
                return 0;
        }

        length<<=1;
        buf = calloc(length, sizeof(*buf));
		assert(buf != NULL);

        /*Create data + complement data for TCS*/
        for (i = 0; i < (unsigned int)argc; i++) {
                if (!get_number(argv[i], &buf[2*i])) {
                        fprintf(stderr, "invalid data (#%d)\n", i + 1);
                        ret = 0;
                        goto done;
                }
                buf[2*i+1]=~buf[2*i];//calculate the complement
        }

        ret = prog_write_tcs(&address, buf, length);
        if (ret) {
                fprintf(stderr, "write to OTP TCS failed: %s (%d)\n", prog_get_err_message(ret), ret);
                ret = 0;
                goto done;
        }
        printf("TCS contents written: \n");
        dump_otp(address, buf, length);
        ret = 1;

done:
        free(buf);

        return ret;
}
int cmdh_boot(int argc, char *argv[])
{
        int ret;

        ret = prog_boot();
        if (ret < 0) {
                fprintf(stderr, "failed to boot: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }

        return 1;
}
int cmdh_read_chip_info(int argc, char *argv[])
{
        chip_info_t chip_info;
        int ret;

        if (argc >= 1 && !strcmp(argv[0], "simple")) {
                char chip_rev[CHIP_REV_STRLEN];
                int ret;

                ret = prog_gdb_read_chip_rev(chip_rev);

                if (ret < 0) {
                        fprintf(stderr, "failed to read chip revision: %s (%d)\n",
                                                                prog_get_err_message(ret), ret);
                        return 0;
                }

                printf("CHIP REVISION: %s\n", chip_rev);

                return 1;
        }

        ret = prog_read_chip_info(&chip_info);
        if (ret < 0) {
                fprintf(stderr, "failed to read chip info: %s (%d)\n", prog_get_err_message(ret), ret);
                return 0;
        }

        printf("CHIP INFO:\n"
                "chip_revision = %s\n"
                "chip_id (as read from otp) = %s\n"
                "chip_package = %s\n",
                chip_info.chip_rev, chip_info.chip_otp_id, chip_info.chip_package);
        return 1;
}

int handle_command(char *cmd, int argc, char *argv[])
{
        struct cli_command *cmdh = cmds;

        /* lookup command handler */
        while (cmdh->name && strcmp(cmdh->name, cmd)) {
                cmdh++;
        }

        /* handlers table is terminated by empty entry, so name == NULL means no handler found */
        if (!cmdh->name) {
                fprintf(stderr, "invalid command\n");
                return 1;
        }

        if (argc < cmdh->min_num_p) {
                fprintf(stderr, "not enough parameters\n");
                return 1;
        }

        /*
         * return value from handler (0=failure) will be used as exit code so need to do change it
         * here to have exit code 0 on success
         */
        return !cmdh->func(argc, argv);
}
