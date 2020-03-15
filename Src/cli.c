/*
 * This file is part of Cleanflight. Modified to DTG project.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "string.h"
#include "stddef.h"
#include "common/utils.h"
#include "stdbool.h"

#include "build/version.h"

#include "common/buf_writer.h"
#include "common/printf.h"
#include "common/string_light.h"
#include "common/typeconversion.h"

#include "drivers/serial.h"

#include "main.h"
#include "cli.h"
#include "settings.h"
#include "parameters.h"
#include "parameters.c"

uint8_t cliMode = 0;
static serialPort_t *cliPort;

static bufWriter_t *cliWriter;

static uint8_t cliWriteBuffer[sizeof(*cliWriter) + 128];

static char cliBuffer[64];

static uint32_t bufferIndex = 0;

static void cliPrint(const char *str)
{
    while (*str) {
        bufWriterAppend(cliWriter, *str++);
    }
}

static void cliPrintLinefeed(void)
{
    cliPrint("\r\n");
}

static void cliPrintLine(const char *str)
{
    cliPrint(str);
    cliPrintLinefeed();
}

static void cliPrintError(const char *str)
{
    cliPrint("### ERROR: ");
    cliPrint(str);
#ifdef USE_CLI_BATCH
    if (commandBatchActive) {
        commandBatchError = true;
    }
#endif
}

static void cliPrintErrorLine(const char *str)
{
    cliPrint("### ERROR: ");
    cliPrintLine(str);
#ifdef USE_CLI_BATCH
    if (commandBatchActive) {
        commandBatchError = true;
    }
#endif
}

#ifdef CLI_MINIMAL_VERBOSITY
#define cliPrintHashLine(str)
#else
static void cliPrintHashLine(const char *str)
{
    cliPrint("\r\n# ");
    cliPrintLine(str);
}
#endif

static void cliPutp(void *p, char ch)
{
    bufWriterAppend(p, ch);
}

static void cliPrintfva(const char *format, va_list va)
{
    tfp_format(cliWriter, cliPutp, format, va);
    bufWriterFlush(cliWriter);
}

static void cliPrintLinefva(const char *format, va_list va)
{
    tfp_format(cliWriter, cliPutp, format, va);
    bufWriterFlush(cliWriter);
    cliPrintLinefeed();
}

/*
static bool cliDumpPrintLinef(uint8_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if (!((dumpMask & DO_DIFF) && equalsDefault)) {
        va_list va;
        va_start(va, format);
        cliPrintLinefva(format, va);
        va_end(va);
        return true;
    } else {
        return false;
    }
}
*/
static void cliWrite(uint8_t ch)
{
    bufWriterAppend(cliWriter, ch);
}

/*
static bool cliDefaultPrintLinef(uint8_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if ((dumpMask & SHOW_DEFAULTS) && !equalsDefault) {
        cliWrite('#');

        va_list va;
        va_start(va, format);
        cliPrintLinefva(format, va);
        va_end(va);
        return true;
    } else {
        return false;
    }
}
*/

static void cliPrintf(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintfva(format, va);
    va_end(va);
}


static void cliPrintLinef(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintLinefva(format, va);
    va_end(va);
}

static void cliPrintErrorVa(const char *format, va_list va)
{
    cliPrint("### ERROR: ");
    cliPrintfva(format, va);
    va_end(va);

#ifdef USE_CLI_BATCH
    if (commandBatchActive) {
        commandBatchError = true;
    }
#endif
}

static void cliPrintErrorLinef(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    cliPrintErrorVa(format, va);
    cliPrintLinefeed();
}

static void printValuePointer(const setting_t *var, const void *valuePointer, uint32_t full)
{
    int32_t value = 0;
    char buf[PARAMETER_MAX_NAME_LENGTH];

    switch (SETTING_TYPE(var)) {
    case VAR_UINT8:
        value = *(uint8_t *)valuePointer;
        break;

    case VAR_INT8:
        value = *(int8_t *)valuePointer;
        break;

    case VAR_UINT16:
        value = *(uint16_t *)valuePointer;
        break;

    case VAR_INT16:
        value = *(int16_t *)valuePointer;
        break;

    case VAR_UINT32:
        value = *(uint32_t *)valuePointer;
        break;

    case VAR_FLOAT:
        cliPrintf("%s", ftoa(*(float *)valuePointer, buf));
        if (full) {
            if (SETTING_MODE(var) == MODE_DIRECT) {
                cliPrintf(" %s", ftoa((float)settingGetMin(var), buf));
                cliPrintf(" %s", ftoa((float)settingGetMax(var), buf));
            }
        }
        return; // return from case for float only

    case VAR_STRING:
        cliPrintf("%s", (const char *)valuePointer);
        return;
    }

    switch (SETTING_MODE(var)) {
    case MODE_DIRECT:
        if (SETTING_TYPE(var) == VAR_UINT32)
            cliPrintf("%u", value);
        else
            cliPrintf("%d", value);
        if (full) {
            if (SETTING_MODE(var) == MODE_DIRECT) {
                cliPrintf(" %d %u", settingGetMin(var), settingGetMax(var));
            }
        }
        break;
    case MODE_LOOKUP:
    {
        const char *name = settingLookupValueName(var, value);
        if (name) {
            cliPrintf(name);
        } else {
            settingGetName(var, buf);
            cliPrintErrorLinef("VALUE %d OUT OF RANGE FOR %s", (int)value, buf);
        }
        break;
    }
    }
}

static bool valuePtrEqualsDefault(const setting_t *value, const void *ptr, const void *ptrDefault)
{
    bool result = false;
    switch (SETTING_TYPE(value)) {
    case VAR_UINT8:
        result = *(uint8_t *)ptr == *(uint8_t *)ptrDefault;
        break;

    case VAR_INT8:
        result = *(int8_t *)ptr == *(int8_t *)ptrDefault;
        break;

    case VAR_UINT16:
        result = *(uint16_t *)ptr == *(uint16_t *)ptrDefault;
        break;

    case VAR_INT16:
        result = *(int16_t *)ptr == *(int16_t *)ptrDefault;
        break;

    case VAR_UINT32:
        result = *(uint32_t *)ptr == *(uint32_t *)ptrDefault;
        break;

    case VAR_FLOAT:
        result = *(float *)ptr == *(float *)ptrDefault;
        break;

    case VAR_STRING:
        result = strncmp(ptr, ptrDefault, settingGetStringMaxLength(value) + 1) == 0;
        break;
    }
    return result;
}

static void cliPrintVar(const setting_t *var, uint32_t full)
{
    const void *ptr = settingGetValuePointer(var);

    printValuePointer(var, ptr, full);
}

static void cliPrintVarRange(const setting_t *var)
{
    switch (SETTING_MODE(var)) {
    case MODE_DIRECT:
        if (SETTING_TYPE(var) == VAR_STRING) {
           cliPrintLinef("Max. length: %u", settingGetStringMaxLength(var));
           break;
        }
        cliPrintLinef("Allowed range: %d - %u", settingGetMin(var), settingGetMax(var));
        break;
    case MODE_LOOKUP:
    {
        const lookupTableEntry_t *tableEntry = settingLookupTable(var);
        cliPrint("Allowed values:");
        for (uint32_t i = 0; i < tableEntry->valueCount ; i++) {
            if (i > 0)
                cliPrint(",");
            cliPrintf(" %s", tableEntry->values[i]);
        }
        cliPrintLinefeed();
    }
        break;
    }
}

typedef union {
    uint32_t uint_value;
    int32_t int_value;
    float float_value;
} int_float_value_t;


static void cliSetIntFloatVar(const setting_t *var, const int_float_value_t value)
{
    void *ptr = settingGetValuePointer(var);

    switch (SETTING_TYPE(var)) {
    case VAR_UINT8:
    case VAR_INT8:
        *(int8_t *)ptr = value.int_value;
        break;

    case VAR_UINT16:
    case VAR_INT16:
        *(int16_t *)ptr = value.int_value;
        break;

    case VAR_UINT32:
        *(uint32_t *)ptr = value.uint_value;
        break;

    case VAR_FLOAT:
        *(float *)ptr = (float)value.float_value;
        break;

    case VAR_STRING:
        // Handled by cliSet directly
        break;
    }
}


static void cliPrompt(void)
{
    cliPrint("\r\n# ");
    bufWriterFlush(cliWriter);
}

static void cliShowParseError(void)
{
    cliPrintErrorLinef("Parse error");
}

static void cliShowArgumentRangeError(char *name, int min, int max)
{
    cliPrintErrorLinef("%s must be between %d and %d", name, min, max);
}

static const char *nextArg(const char *currentArg)
{
    const char *ptr = strchr(currentArg, ' ');
    while (ptr && *ptr == ' ') {
        ptr++;
    }

    return ptr;
}

// Check if a string's length is zero
static bool isEmpty(const char *string)
{
    return (string == NULL || *string == '\0') ? true : false;
}

typedef struct {
    const char *name;
#ifndef SKIP_CLI_COMMAND_HELP
    const char *description;
    const char *args;
#endif
    void (*func)(char *cmdline);
} clicmd_t;

/*
static const char *checkCommand(const char *cmdLine, const char *command)
{
    if (!sl_strncasecmp(cmdLine, command, strlen(command))   // command names match
        && !sl_isalnum((unsigned)cmdLine[strlen(command)])) {   // next characted in bufffer is not alphanumeric (command is correctly terminated)
        return cmdLine + strlen(command) + 1;
    } else {
        return 0;
    }
}
*/
static void cliReboot(void)
{
    cliPrint("\r\nRebooting");
    bufWriterFlush(cliWriter);
    //waitForSerialPortToFinishTransmitting(cliPort);
}

static void cliExit(char *cmdline)
{
    UNUSED(cmdline);

#ifndef CLI_MINIMAL_VERBOSITY
    cliPrintLine("\r\nLeaving CLI mode, unsaved changes lost.");
#endif
    bufWriterFlush(cliWriter);

    *cliBuffer = '\0';
    bufferIndex = 0;
    cliMode = 0;
    //cliReboot();

    cliWriter = NULL;
}
static void cliGet(char *cmdline)
{

    const setting_t *val;
    int matchedCommands = 0;
    char name[PARAMETER_MAX_NAME_LENGTH];

    while(*cmdline == ' ') ++cmdline; // ignore spaces

    for (uint32_t i = 0; i < ARRAYLEN(parametersTable); i++) {
        val = settingGet(i);
        if (settingNameContains(val, name, cmdline)) {
            cliPrintf("%s = ", name);
            cliPrintVar(val, 0);
            cliPrintLinefeed();
            cliPrintVarRange(val);
            cliPrintLinefeed();

            matchedCommands++;
        }
    }


    if (matchedCommands) {
        return;
    }

    cliPrintErrorLine("Invalid name");
}

static void cliSet(char *cmdline)
{
	UNUSED(cmdline);
	/*
    uint32_t len;
    const setting_t *val;
    char *eqptr = NULL;
    char name[SETTING_MAX_NAME_LENGTH];

    while(*cmdline == ' ') ++cmdline; // ignore spaces

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        cliPrintLine("Current settings:");
        for (uint32_t i = 0; i < SETTINGS_TABLE_COUNT; i++) {
            val = settingGet(i);
            settingGetName(val, name);
            cliPrintf("%s = ", name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            cliPrintLinefeed();
        }
    } else if ((eqptr = strstr(cmdline, "=")) != NULL) {
        // has equals

        char *lastNonSpaceCharacter = eqptr;
        while (*(lastNonSpaceCharacter - 1) == ' ') {
            lastNonSpaceCharacter--;
        }
        uint8_t variableNameLength = lastNonSpaceCharacter - cmdline;

        // skip the '=' and any ' ' characters
        eqptr++;
        while (*(eqptr) == ' ') {
            eqptr++;
        }

        for (uint32_t i = 0; i < SETTINGS_TABLE_COUNT; i++) {
            val = settingGet(i);
            // ensure exact match when setting to prevent setting variables with shorter names
            if (settingNameIsExactMatch(val, name, cmdline, variableNameLength)) {
                const setting_type_e type = SETTING_TYPE(val);
                if (type == VAR_STRING) {
                    settingSetString(val, eqptr, strlen(eqptr));
                    return;
                }
                const setting_mode_e mode = SETTING_MODE(val);
                bool changeValue = false;
                int_float_value_t tmp = {0};
                switch (mode) {
                case MODE_DIRECT: {
                        if (*eqptr != 0 && strspn(eqptr, "0123456789.+-") == strlen(eqptr)) {
                            float valuef = fastA2F(eqptr);
                            // note: compare float values
                            if (valuef >= (float)settingGetMin(val) && valuef <= (float)settingGetMax(val)) {

                                if (type == VAR_FLOAT)
                                    tmp.float_value = valuef;
                                else if (type == VAR_UINT32)
                                    tmp.uint_value = fastA2UL(eqptr);
                                else
                                    tmp.int_value = fastA2I(eqptr);

                                changeValue = true;
                            }
                        }
                    }
                    break;
                case MODE_LOOKUP: {
                        const lookupTableEntry_t *tableEntry = settingLookupTable(val);
                        bool matched = false;
                        for (uint32_t tableValueIndex = 0; tableValueIndex < tableEntry->valueCount && !matched; tableValueIndex++) {
                            matched = sl_strcasecmp(tableEntry->values[tableValueIndex], eqptr) == 0;

                            if (matched) {
                                tmp.int_value = tableValueIndex;
                                changeValue = true;
                            }
                        }
                    }
                    break;
                }

                if (changeValue) {
                    cliSetIntFloatVar(val, tmp);

                    cliPrintf("%s set to ", name);
                    cliPrintVar(val, 0);
                } else {
                    cliPrintError("Invalid value. ");
                    cliPrintVarRange(val);
                    cliPrintLinefeed();
                }

                return;
            }
        }
        cliPrintErrorLine("Invalid name");
    } else {
        // no equals, check for matching variables.
        cliGet(cmdline);
    }
    */
}

static void cliStatus(char *cmdline)
{
    UNUSED(cmdline);

    //char buf[MAX(FORMATTED_DATE_TIME_BUFSIZE, SETTING_MAX_NAME_LENGTH)];
    //dateTime_t dt;

    //cliPrintLinef("System Uptime: %d seconds", millis() / 1000);
    //rtcGetDateTime(&dt);
    //dateTimeFormatLocal(buf, &dt);
    //cliPrintLinef("Current Time: %s", buf);
    cliPrintf("CPU Clock=%dMHz", (SystemCoreClock / 1000000));

    cliPrintLinefeed();

    cliPrintLine("STM32 system clocks:");
#if defined(USE_HAL_DRIVER)
    cliPrintLinef("  SYSCLK = %d MHz", HAL_RCC_GetSysClockFreq() / 1000000);
    cliPrintLinef("  HCLK   = %d MHz", HAL_RCC_GetHCLKFreq() / 1000000);
    cliPrintLinef("  PCLK1  = %d MHz", HAL_RCC_GetPCLK1Freq() / 1000000);
    cliPrintLinef("  PCLK2  = %d MHz", HAL_RCC_GetPCLK2Freq() / 1000000);
#else
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    cliPrintLinef("  SYSCLK = %d MHz", clocks.SYSCLK_Frequency / 1000000);
    cliPrintLinef("  HCLK   = %d MHz", clocks.HCLK_Frequency / 1000000);
    cliPrintLinef("  PCLK1  = %d MHz", clocks.PCLK1_Frequency / 1000000);
    cliPrintLinef("  PCLK2  = %d MHz", clocks.PCLK2_Frequency / 1000000);
#endif
}

static void cliVersion(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintLinef("# %s/%s %s %s / %s (%s)",
        FC_FIRMWARE_NAME,
        targetName,
        FC_VERSION_STRING,
        buildDate,
        buildTime,
        shortGitRevision
    );
    cliPrintLinef("# GCC-%s",
        compilerVersion
    );
}

static void cliSave(char *cmdline)
{
    UNUSED(cmdline);

#ifdef USE_CLI_BATCH
    if (commandBatchActive && commandBatchError) {
        cliPrintCommandBatchWarning("PLEASE FIX ERRORS THEN 'SAVE'");
        resetCommandBatch();
        return;
    }
#endif

    cliPrint("Saving");
    //copyCurrentProfileToProfileSlot(getConfigProfile();
    //writeEEPROM();
    cliReboot();
}

static void cliDefaults(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("Resetting to defaults");
    //resetEEPROM();

#ifdef USE_CLI_BATCH
    commandBatchError = false;
#endif

    //if (!checkCommand(cmdline, "noreboot"))
        //cliReboot();
}

#ifndef SKIP_CLI_COMMAND_HELP
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name , \
    description , \
    args , \
    method \
}
#else
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name, \
    method \
}
#endif

static void cliHelp(char *cmdline);

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
	CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", NULL, cliDefaults),
    CLI_COMMAND_DEF("exit", NULL, NULL, cliExit),
    CLI_COMMAND_DEF("get", "get variable value", "[name]", cliGet),
    CLI_COMMAND_DEF("help", NULL, NULL, cliHelp),
    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
    CLI_COMMAND_DEF("set", "change setting", "[<name>=<value>]", cliSet),
    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),
};

static void cliHelp(char *cmdline)
{
    UNUSED(cmdline);

    for (uint32_t i = 0; i < ARRAYLEN(cmdTable); i++) {
        cliPrint(cmdTable[i].name);
#ifndef SKIP_CLI_COMMAND_HELP
        if (cmdTable[i].description) {
            cliPrintf(" - %s", cmdTable[i].description);
        }
        if (cmdTable[i].args) {
            cliPrintf("\r\n\t%s", cmdTable[i].args);
        }
#endif
        cliPrintLinefeed();
    }
}

void cliProcess(void)
{
    if (!cliWriter) {
        return;
    }

    // Be a little bit tricky.  Flush the last inputs buffer, if any.
    bufWriterFlush(cliWriter);

    while (serialRxBytesWaiting(cliPort)) {
        uint8_t c = serialRead(cliPort);
        if (c == '\t' || c == '?') {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            uint32_t i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
                if (bufferIndex && (sl_strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart) {    /* Buffer matches one or more commands */
                for (; ; bufferIndex++) {
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                        break;
                    if (!pstart->name[bufferIndex] && bufferIndex < sizeof(cliBuffer) - 2) {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        cliBuffer[bufferIndex] = '\0';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend) {
                /* Print list of ambiguous matches */
                cliPrint("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++) {
                    cliPrint(cmd->name);
                    cliWrite('\t');
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                cliWrite(cliBuffer[i]);
        } else if (!bufferIndex && c == 4) {   // CTRL-D
            cliExit(cliBuffer);
            return;
        } else if (c == 12) {                  // NewPage / CTRL-L
            // clear screen
            cliPrint("\033[2J\033[1;1H");
            cliPrompt();
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            cliPrintLinefeed();

            // Strip comment starting with # from line
            char *p = cliBuffer;
            p = strchr(p, '#');
            if (NULL != p) {
                bufferIndex = (uint32_t)(p - cliBuffer);
            }

            // Strip trailing whitespace
            while (bufferIndex > 0 && cliBuffer[bufferIndex - 1] == ' ') {
                bufferIndex--;
            }

            // Process non-empty lines
            if (bufferIndex > 0) {
                cliBuffer[bufferIndex] = 0; // null terminate

                const clicmd_t *cmd;
                for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
                    if (!sl_strncasecmp(cliBuffer, cmd->name, strlen(cmd->name))   // command names match
                       && !sl_isalnum((unsigned)cliBuffer[strlen(cmd->name)]))    // next characted in bufffer is not alphanumeric (command is correctly terminated)
                        break;
                }
                if (cmd < cmdTable + ARRAYLEN(cmdTable))
                    cmd->func(cliBuffer + strlen(cmd->name) + 1);
                else
                    cliPrintError("Unknown command, try 'help'");
                bufferIndex = 0;
            }

            memset(cliBuffer, 0, sizeof(cliBuffer));

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode)
                return;

            cliPrompt();
        } else if (c == 127) {
            // backspace
            if (bufferIndex) {
                cliBuffer[--bufferIndex] = 0;
                cliPrint("\010 \010");
            }
        } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            if (!bufferIndex && c == ' ')
                continue; // Ignore leading spaces
            cliBuffer[bufferIndex++] = c;
            cliWrite(c);
        }
    }
}

void cliEnter(serialPort_t *serialPort)
{
    if (cliMode) {
        return;
    }

    cliMode = 1;
    cliPort = serialPort;
    setPrintfSerialPort(cliPort);
    cliWriter = bufWriterInit(cliWriteBuffer, sizeof(cliWriteBuffer), (bufWrite_t)serialWriteBufShim, serialPort);

#ifndef CLI_MINIMAL_VERBOSITY
    cliPrintLine("\r\nEntering CLI Mode, type 'exit' to return, or 'help'");
#else
    cliPrintLine("\r\nCLI");
#endif
    cliPrompt();

#ifdef USE_CLI_BATCH
    resetCommandBatch();
#endif

}
/*
void cliInit(const serialConfig_t *serialConfig)
{
    UNUSED(serialConfig);
}
*/
