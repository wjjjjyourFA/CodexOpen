#include "hc_msg_parser.h"

#include <assert.h>
#include <unistd.h>

/**
 * @brief reset a token object
 *
 * @param token a token object
 * */
void hc__msg_reset_token(hc__msg_token_t *token)
{
    assert(token); /* Non-NULL token object expected. */

    memset(token, 0, sizeof(hc__msg_token_t));
}

/**
 * @brief initialize a parser
 *
 * @param parser
 * @return int @c 0 if the function succeeded, @c others on error.
 * */
int hc__init_msg_parser(hc__msg_parser_t *parser, size_t buffer_size, hc__msg_read_handler_f read_handler, void *parser_id)
{
    assert(parser);      /* Non-NULL token object expected. */
    assert(buffer_size); /* None 0 expected. */

    parser->error.error_code = HC__STATE_NONE;
    strncpy(parser->error.description, "No error", 9);

    parser->state = HC__STATE_S0;
    // 初始化的方式 设置 parser->state
    // 疑问，HC__STATE_S0 对应哪一个协议？

    parser->buffer.end = buffer_size;
    parser->buffer.pointer = 0;
    parser->buffer.last = 0;
    parser->buffer.value = (void *)malloc(buffer_size);
    parser->parser_id = parser_id;
    parser->read_handler = read_handler;

    return 0;
}

/**
 * Functions to deal all state
 * return value means pointer move steps
 * */
static int hc__msg_deal_S0(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S1(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S2(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S3(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S4(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S5(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S6(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S7(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S8(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S9(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S10(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S11(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S12(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S13(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S14(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S15(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S16(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S17(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S18(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S19(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S20(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S21(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S22(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S23(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);
static int hc__msg_deal_S24(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c);

// bind state with proc func
/*
这段代码创建了一个函数指针数组 g_hc__msg_state_handler，
数组的每个元素都对应着一个特定的状态（如 HC__STATE_S0、HC__STATE_S1 等），
并且与一个函数关联起来（比如 hc__msg_deal_S0、hc__msg_deal_S1 等）。

这种结构通常用于状态机或类似系统中，其中不同的状态对应着不同的操作或行为。
通过这个数组，你可以根据系统的当前状态调用相应的函数。
例如，如果系统处于状态 HC__STATE_S0，
你可以调用 g_hc__msg_state_handler[HC__STATE_S0] 来执行函数 hc__msg_deal_S0。
这种方式使得状态机的逻辑结构清晰、模块化。
*/ 
static int (*g_hc__msg_state_handler[])(hc__msg_parser_t *, hc__msg_token_t *, unsigned char) = {
    [HC__STATE_S0] = hc__msg_deal_S0,
    [HC__STATE_S1] = hc__msg_deal_S1,
    [HC__STATE_S2] = hc__msg_deal_S2,
    [HC__STATE_S3] = hc__msg_deal_S3,
    [HC__STATE_S4] = hc__msg_deal_S4,
    [HC__STATE_S5] = hc__msg_deal_S5,
    [HC__STATE_S6] = hc__msg_deal_S6,
    [HC__STATE_S7] = hc__msg_deal_S7,
    [HC__STATE_S8] = hc__msg_deal_S8,
    [HC__STATE_S9] = hc__msg_deal_S9,
    [HC__STATE_S10] = hc__msg_deal_S10,
    [HC__STATE_S11] = hc__msg_deal_S11,
    [HC__STATE_S12] = hc__msg_deal_S12,
    [HC__STATE_S13] = hc__msg_deal_S13,
    [HC__STATE_S14] = hc__msg_deal_S14,
    [HC__STATE_S15] = hc__msg_deal_S15,
    [HC__STATE_S16] = hc__msg_deal_S16,
    [HC__STATE_S17] = hc__msg_deal_S17,
    [HC__STATE_S18] = hc__msg_deal_S18,
    [HC__STATE_S19] = hc__msg_deal_S19,
    [HC__STATE_S20] = hc__msg_deal_S20,
    [HC__STATE_S21] = hc__msg_deal_S21,
    [HC__STATE_S22] = hc__msg_deal_S22,
    [HC__STATE_S23] = hc__msg_deal_S23,
    [HC__STATE_S24] = hc__msg_deal_S24,
};

/**
 * Scan the input stream and produce the next token.
 *
 * Call the function subsequently to produce a sequence of tokens corresponding
 * to the input stream.
 *
 * An application is responsible for freeing any buffers associated with the
 * produced token object using the @c hsl__file_token_delete function.
 *
 * @param[in,out]   parser      A parser object.
 * @param[out]      token       An token object.
 *
 * @returns @c 0 if the function succeeded, @c -1 on error.
 */

int hc__msg_parser_scan(hc__msg_parser_t *parser, hc__msg_token_t *token)
{
    assert(parser); /* Non-NULL parser object is expected. */
    assert(token);  /* Non-NULL token object is expected. */

    hc__msg_reset_token(token);  // ==> 0

    if (parser->error.error_code != HC__STATE_NONE)
        return -1;

    while (1)
    {   // 一旦有数据可读，它将从缓冲区中取出下一个字符，并将其存储在变量 c 中
        // 该循环将 parser->buffer.value 中的字符一个个取出判断，直到一个完整的令牌被解析出来。

        if (parser->buffer.pointer == parser->buffer.last)
        {   // 如果指针等于末尾，说明缓冲区为空，需要从外部读取数据到缓冲区中。
            // 然后调用 parser->read_handler 从外部读取数据到缓冲区中，并等待至少读取到一个字符。
            // 接着重置指针并进入一个循环。

            memset((void *)parser->buffer.value, 0x00, parser->buffer.end);
            parser->read_handler((void *)parser->buffer.value, parser->buffer.end, &parser->buffer.last, parser->parser_id);
            parser->buffer.pointer = 0;
            // wait until at least one char read
            while (parser->buffer.last <= 0)  // 至少读取到一个字符
            {
                // 读取数据到缓存中
                parser->read_handler((void *)parser->buffer.value, parser->buffer.end, &parser->buffer.last, parser->parser_id);
                // parser->buffer.value 存储读取的数据
                // parser->buffer.end   读取多少字节的数据
                // parser->buffer.last  已读取到哪个内存地址
                // parser->parser_id    传递解析器的参数
                usleep(1000);
            }  
        }

        // 对读取到的这一个字符进行处理
        unsigned char c = ((unsigned char *)parser->buffer.value)[parser->buffer.pointer];

        // deal all state. call func pointer
        if (parser->state <= HC__STATE_S24 && parser->state >= HC__STATE_S0)
        {
            // 处理函数会返回一个表示处理完一个字符后应该移动的步数。
            int pointer_move_step = g_hc__msg_state_handler[parser->state](parser, token, c);
            // pointer_move_step == 1  ==>  步进计数，代表一个字符被正确处理

            int new_token_value_len = token->token_value.value_len + pointer_move_step;
            int new_token_pointer = parser->buffer.pointer + pointer_move_step;  
            // printf("token->token_value.value_len: %d\n", token->token_value.value_len);  // 0 -> 208
            // printf("pointer_move_step: %d\n", pointer_move_step);
            // printf("new_token_value_len: %d\n", new_token_value_len);
            // printf("parser->buffer.pointer: %ld\n", parser->buffer.pointer);
            // printf("new_token_pointer: %d\n", new_token_pointer);

            // printf("sizeof(token->token_value.value): %ld\n", sizeof(token->token_value.value));
            // 如果令牌对象的值长度超出了预设的最大长度，则将令牌类型设置为错误类型
            if (new_token_value_len < sizeof(token->token_value.value))  // ==> 1 < 1024 ???
            {
                memcpy(token->token_value.value + token->token_value.value_len, parser->buffer.value + parser->buffer.pointer, pointer_move_step);
                token->token_value.value_len = new_token_value_len;
                parser->buffer.pointer = new_token_pointer;
            }
            else
            {
                token->type = HC__TOKERN_ERROR;
                parser->error.error_code = parser->state;
                snprintf(parser->error.description, sizeof(parser->error.description), "token value too long [%d]", token->token_value.value_len);
            }
        }
        else
        {
            fprintf(stderr, "state S[%d]", parser->state);
            assert(0);
        }

        if (token->type != HC__TOKERN_NONE)
            break;
    }

    return 0;
}

/* *******************************************************************
 * Functions dealing state
 * ***************************************************************** */

static int hc__msg_deal_S0(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0xaa == c)
    {
        parser->state = HC__STATE_S1;
    }
    else if ('$' == c)
    {
        parser->state = HC__STATE_S18;
        parser->specified_len = 5;
        // 检测到标志位，$GPCHC  
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S0;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);
    }

    return 1;
}

static int hc__msg_deal_S1(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0xcc == c)
    {
        parser->state = HC__STATE_S2;  // HC__TOKERN_HC_MSG
    }
    else if (0x55 == c)
    {
        parser->state = HC__STATE_S12;  // HC__TOKERN_HC_SHORT_MSG
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S1;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S2(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0x48 == c)
    {
        parser->state = HC__STATE_S3;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S2;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S3(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0x43 == c)
    {
        parser->state = HC__STATE_S4;
        parser->specified_len = 2;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S3;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S4(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S5;
        parser->specified_len = 2;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S5(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S6;
        parser->specified_len = 2;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S6(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S7;
        parser->specified_len = 4;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S7(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S8;
        parser->specified_len = 4;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S8(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S9;
        parser->specified_len = 4;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S9(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S10;
        unsigned char hc_msg_len_l = token->token_value.value[4];
        unsigned char hc_msg_len_h = token->token_value.value[5];
        parser->specified_len = (hc_msg_len_h << 8) + hc_msg_len_l + 4;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S10(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {      
        parser->specified_len--;
    }

    if (parser->specified_len == 0)
    {
        token->type = HC__TOKERN_HC_MSG;
        parser->state = HC__STATE_S0;
    }

    return 1;
}

static int hc__msg_deal_S11(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    token->type = HC__TOKERN_HC_MSG;
    parser->state = HC__STATE_S0;
    return 0;
}

static int hc__msg_deal_S12(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0x48 == c)
    {
        parser->state = HC__STATE_S13;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S12;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S13(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (0x43 == c)
    {
        parser->state = HC__STATE_S14;
        parser->specified_len = 2;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S13;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S14(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S15;
        parser->specified_len = 2;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S15(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }
    else
    {
        parser->state = HC__STATE_S16;
        unsigned char hc_msg_len_l = token->token_value.value[4];
        unsigned char hc_msg_len_h = token->token_value.value[5];
        parser->specified_len = (hc_msg_len_h << 8) + hc_msg_len_l + 4;

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S16(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
    }

    if (parser->specified_len == 0)
    {
        token->type = HC__TOKERN_HC_SHORT_MSG;
        parser->state = HC__STATE_S0;
    }

    return 1;
}

static int hc__msg_deal_S17(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    token->type = HC__TOKERN_HC_SHORT_MSG;
    parser->state = HC__STATE_S0;

    return 0;
}

static int hc__msg_deal_S18(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
        // 执行五次
    }
    else
    {
        parser->state = HC__STATE_S19;
        // 赋值操作在外部通过缓冲区转到token实现 
        // 此处进过5次循环，实际上将 GPCHC 复制进token

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S19(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    char nmea_head_2[2];
    // char nmea_head_2[6];

    nmea_head_2[0] = token->token_value.value[1];
    nmea_head_2[1] = token->token_value.value[2];
    // strncpy(nmea_head_2, token->token_value.value, 6);
    // printf("nmea_head_2: %s\n", nmea_head_2);

/*
    // 字符数组的该种赋值方法，没有添加截止位 '\0'
    // 导致 nmea_head_2 GP ==> GP$GPCH
    // Copy first five characters of token value to nmea_head_wj
    char nmea_head_wj[5];
    // strncpy(nmea_head_wj, token->token_value.value, 5);
    strncpy(nmea_head_wj, token->token_value.value, 5);
    nmea_head_wj[5] = '\0'; // Null terminate the string
    printf("nmea_head_wj: %s\n", nmea_head_wj);
    printf("nmea_head_2: %s\n", nmea_head_2);
*/

    if (strcmp(nmea_head_2, "GP") == 0 || strcmp(nmea_head_2, "GN") == 0 || strcmp(nmea_head_2, "BD") == 0 || strcmp(nmea_head_2, "GT") == 0)
    {
        // printf("nmea_head_2: %s\n", nmea_head_2);
        if (',' == c) {
            parser->state = HC__STATE_S21;
            // $GPCHC 以 , 分割
            // printf("HC__STATE_S21  nmea_head_2: %s\n", nmea_head_2);
        }
        else {
            parser->state = HC__STATE_S20;
            // printf("HC__STATE_S20  nmea_head_2: %s\n", nmea_head_2);
        }
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S19;
        snprintf(parser->error.description, sizeof(parser->error.description), "unsupport nmea head [$%s]", nmea_head_2);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S20(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (',' == c)
    {
        parser->state = HC__STATE_S21;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S20;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S21(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if ('*' == c)
    {
        parser->state = HC__STATE_S22;
        parser->specified_len = 4;
    }
    else if ((c <= 'Z' && c >= 'A') || (c <= 'z' && c >= 'a') || (c <= '9' && c >= '0') || (',' == c) || ('.' == c) || ('-' == c))
    {
        // 此处未改变 state , 所以将不断将字符从缓冲压入token
        return 1;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S21;
        snprintf(parser->error.description, sizeof(parser->error.description), "error char[0x%02x][%c]", c, c);

        return 0;
    }

    return 1;
}

static int hc__msg_deal_S22(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    if (parser->specified_len > 0)
    {
        parser->specified_len--;
        // 执行四次
        // 确保全部字符都被转移到 toekn ?
    }

    if (parser->specified_len == 0)
    {
        token->type = HC__TOKERN_NMEA_MGS;
        parser->state = HC__STATE_S0;
    }

    return 1;
}

static int hc__msg_deal_S23(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    char nmea_end_2[2];

    nmea_end_2[0] = token->token_value.value[token->token_value.value_len - 2];
    nmea_end_2[1] = token->token_value.value[token->token_value.value_len - 1];

    if (strcmp(nmea_end_2, "\r\n") == 0)
    {
        parser->state = HC__STATE_S24;
    }
    else
    {
        token->type = HC__TOKERN_ERROR;
        parser->error.error_code = HC__STATE_S23;
        snprintf(parser->error.description, sizeof(parser->error.description), "error nmea end [0x%x 0x%x]", nmea_end_2[0], nmea_end_2[1]);

        return 0;
    }

    return 0;
}

static int hc__msg_deal_S24(hc__msg_parser_t *parser, hc__msg_token_t *token, unsigned char c)
{
    token->type = HC__TOKERN_NMEA_MGS;
    parser->state = HC__STATE_S0;

    return 0;
}