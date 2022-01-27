/*
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                                  Martin Jacquet - June 2020
 */
#include "acarucotag.h"

#include "arucotag_c_types.h"

#include "codels.hpp"

/* --- Attribute output_frame ------------------------------------------- */

/** Validation codel output_frame of attribute output_frame.
 *
 * Returns genom_ok.
 * Throws arucotag_e_sys.
 */
genom_event
output_frame(int16_t out_frame, const genom_context self)
{
    if (out_frame < 0 || out_frame > 2) {
        warnx("wrong output frame value (allowed: 0, 1, 2)");
        errno = EDOM;
        return arucotag_e_sys_error("wrong value", self);
    }
    return genom_ok;
}

/* --- Function log ----------------------------------------------------- */

/** Codel log_start of function log.
 *
 * Returns genom_ok.
 * Throws arucotag_e_sys.
 */
genom_event
log_start(const char path[64], uint32_t decimation,
          arucotag_log_s **log, const genom_context self)
{
    int fd;

    fd = open(path, O_WRONLY|O_APPEND|O_CREAT|O_TRUNC, 0666);
    if (fd < 0) return arucotag_e_sys_error(path, self);

    if (write(fd, arucotag_log_header "\n", sizeof(arucotag_log_header)) < 0)
        return arucotag_e_sys_error(path, self);

    if ((*log)->req.aio_fildes >= 0)
    {
        close((*log)->req.aio_fildes);
        if ((*log)->pending)
            while (aio_error(&(*log)->req) == EINPROGRESS)
              /* empty body */;
    }
    (*log)->req.aio_fildes = fd;
    (*log)->pending = false;
    (*log)->skipped = false;
    (*log)->decimation = decimation < 1 ? 1 : decimation;
    (*log)->missed = 0;
    (*log)->total = 0;
    warnx("logging started to %s", path);
    return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
log_stop(arucotag_log_s **log, const genom_context self)
{
    if (*log && (*log)->req.aio_fildes >= 0)
        close((*log)->req.aio_fildes);
    (*log)->req.aio_fildes = -1;

    warnx("logging terminated");
    return genom_ok;
}


/* --- Function log_info ------------------------------------------------ */

/** Codel log_info of function log_info.
 *
 * Returns genom_ok.
 */
genom_event
log_info(const arucotag_log_s *log, uint32_t *miss, uint32_t *total,
         const genom_context self)
{
    *miss = *total = 0;
    if (log) {
        *miss = log->missed;
        *total = log->total;
    }
    return genom_ok;
}
