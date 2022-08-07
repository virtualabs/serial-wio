#include "SAMDTimerInterrupt.h"
#include"TFT_eSPI.h"

#define BUZZER_PIN WIO_BUZZER /* sig pin of the buzzer */

#define MAX_MEASURES    5000
#define IDLE_INTERVAL   100000
#define PROBE_INPUT     PIN_SERIAL1_RX
#define SCREEN_WIDTH    50
#define SCREEN_HEIGHT   22
#define LINE_HEIGHT     10
#define LOG_OFFSET_Y    (2*LINE_HEIGHT)

#define BG_COLOR        TFT_BLACK
#define TEXT_COLOR      TFT_LIGHTGREY

/*********************
 * Typedefs
 *********************/

typedef enum {
  WAITING,
  ACQUIRE,
  ANALYZE,
  MONITOR
} mp_state_t;

enum {
  PAR_NONE = 0,
  PAR_ODD,
  PAR_EVEN,
  PAR_MAX
};

typedef struct {
  int period;
  int baudrate;
} baudrate_info_t;

/**********************
 * Globals
 **********************/

/* Global state. */
volatile mp_state_t g_state;

/* Logic analysis. */
static uint32_t g_measures[MAX_MEASURES];
volatile uint32_t g_usec_counter;
volatile uint32_t g_last_timestamp;
volatile int g_nb_measures;
volatile int baudrate = 0, uart_config, good_par, g_bytesize;

/* Timer. */
SAMDTimer ITimer0(TIMER_TC3);

/* Screen. */
TFT_eSPI tft;
char g_uart_log[SCREEN_HEIGHT][SCREEN_WIDTH+1];
int g_uart_cur_col, g_uart_cur_line, g_uart_top_line;



/**********************************************
 **********************************************
 *
 * Helpers
 * 
 **********************************************
 **********************************************/

/**
 * @brief Convert period to standard baudrate
 * @param period: measured period in microseconds
 * @return closest standard baudrate
 */
int period_to_baudrate(int period)
{
  int i=0;
  baudrate_info_t rates[] = {
    {4, 230400},
    {8, 115200},
    {13, 76800},
    {17, 57600},
    {26, 38400},
    {34, 28800},
    {52, 19200},
    {104, 9600},
    {0, 0}
  };

  while (rates[i].period > 0)
  {
    if ( period <= (rates[i].period+1))
    {
      return rates[i].baudrate;
    }
    i++;
  }

  /* Not found, return -1. */
  return -1;
}


/**
 * @brief Count printable characters
 * @param p_data: pointer to an array of characters
 * @param data_size: number of characters in the given array
 * @return number of printable characters
 */

int count_printable_chars(char *p_data, int data_size)
{
  int count=0;
  for (int i=0; i<data_size; i++)
  {
    if (isPrintable(p_data[i]))
      count++;
  }

  return count;
}


/**
 * @brief uSec counter callback (used for logic analysis).
 **/

void u_sec_counter(void)
{
  /* Increment our counter. */
  g_usec_counter++;

  /* Detect if we detect an idling UART. */
  if ((g_state == ACQUIRE) && ((g_usec_counter - g_last_timestamp) > IDLE_INTERVAL))
  {
    g_state = ANALYZE;
  }
}

/**
 * @brief Play tone.
 * @param tone: tone to play (Hz)
 * @param duration: tone duration (milliseconds)
 **/

void playTone(int tone, int duration) {
    for (long i = 0; i < duration * 1000L; i += tone * 2) {
        digitalWrite(BUZZER_PIN, HIGH);
        delayMicroseconds(tone);
        digitalWrite(BUZZER_PIN, LOW);
        delayMicroseconds(tone);
    }
}


/**********************************************
 **********************************************
 *
 * Screen management
 * 
 **********************************************
 **********************************************/

/**
 * @brief Clear UART log screen
 */

void clear_log_screen(void)
{
  /* Clear screen buffer. */
  memset(g_uart_log, 0, (SCREEN_WIDTH+1)*SCREEN_HEIGHT);
  g_uart_cur_line = 0;
  g_uart_cur_col = 0;
  g_uart_top_line = 0;

  /* Clear screen. */
  tft.fillRect(0, LOG_OFFSET_Y, 320, 240 - LOG_OFFSET_Y, BG_COLOR);
}


/**
 * @brief Update status bar (in MONITOR mode)
 */
 
void update_status_bar(void)
{
  char parity[] = "NOE";
  char sz_baudrate[10];
  char sz_config[4];
  snprintf(sz_baudrate, 10, "%d", baudrate);
  snprintf(sz_config, 4, "%d%c1", g_bytesize, parity[good_par]);
  
  tft.fillRect(0, 0, 320, LOG_OFFSET_Y-2, TFT_DARKGREEN);
  tft.setTextSize(2);
  tft.drawString(sz_baudrate, 5, 2);
  tft.drawString(sz_config, 90, 2);
  tft.setTextSize(1);
}


/**
 * @brief Update measures bar (in WAITING mode)
 */
 
void update_waiting_measures(void)
{
  int prog = (g_nb_measures * 150.0)/MAX_MEASURES;
  tft.drawRect(85, 125, 150, 10, TFT_NAVY);
  tft.fillRect(86, 126, prog,8, TFT_NAVY);
}


/**
 * @brief Update our waiting screen (in WAITING mode)
 */
 
void wait_screen(void)
{
  tft.fillRect(0, 0, 320, 240,BG_COLOR);
  tft.fillRect(80, 90, 160, 60, TFT_DARKGREY);
  tft.setTextSize(2);
  tft.drawString("Syncing ...", 85, 95);
  tft.setTextSize(1);

  update_waiting_measures();
}


/**********************************************
 **********************************************
 *
 * User button callbacks
 * 
 **********************************************
 **********************************************/

/**
 * @brief Resync button callback
 **/
 
void resync_button_pressed(void)
{
  /* No effect if we are currently syncing ... */
  if (g_state == WAITING)
    return;
  
  /* Debounce */
  delay(10);
  if (digitalRead(WIO_KEY_C) != 0)
    return;
  
  /* Setup our input GPIO. */
  Serial1.end();
  attachInterrupt(digitalPinToInterrupt(PROBE_INPUT), track_state, CHANGE);
  
  /* Restart analysis. */
  g_state = WAITING;

  /* Reset measures. */
  g_nb_measures = 0;

  /* Reset counter. */
  g_usec_counter = 0;
  g_last_timestamp = 0;

  /* Reset screen. */
  wait_screen();
}


/**
 * @brief Clear screen button callback
 **/

void clear_button_pressed(void)
{
  if (g_state == MONITOR)
  {
    /* Debounce */
    delay(10);
    if (digitalRead(WIO_KEY_A) != 0)
      return;

    clear_log_screen();
  }
}


/**********************************************
 **********************************************
 *
 * Signal acquisition callback (timer)
 * 
 **********************************************
 **********************************************/

/**
 * @brief Track state of input probe.
 * 
 * Fills g_measures with timestamps.
 */

void track_state(void)
{
  switch(g_state)
  {
    case WAITING:
      {
        /* Found a falling edge, start acquisition. */
        if (digitalRead(PROBE_INPUT) == 0)
        {
          /* Save measure. */
          g_nb_measures = 0;
          g_measures[g_nb_measures++] = g_usec_counter;
          g_last_timestamp = g_usec_counter;
          
          /* Set mode to ACQUIRE. */
          g_state = ACQUIRE;
        }
      }
      break;

    case ACQUIRE:
      {
        /* Save measure if we have enough memory. */
        if (g_nb_measures < MAX_MEASURES)
        {
          /* Save measure. */
          g_measures[g_nb_measures++] = g_usec_counter;
          g_last_timestamp = g_usec_counter;
        }
        else
        {
          Serial.println("> Switching to analysis");
          /* Switch to analyze mode. */
          g_state = ANALYZE;
        }
      }
      break;
  }
}
 

/**********************************************
 **********************************************
 *
 * Main code
 * 
 **********************************************
 **********************************************/

/**
 * @brief Setup routine (initializes the WioTerminal)
 */
 
void setup() {
  /* Setup WioTerminal Serial. */
  Serial.begin(115200);

  /* Setup screen. */
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(BG_COLOR); //Red background
  tft.setTextColor(TEXT_COLOR);          //sets the text colour to black
  tft.setTextSize(1);                   //sets the size of text

  /* Clear screen. */
  clear_log_screen();

  /* Setup our buttons. */
  pinMode(WIO_KEY_A, INPUT_PULLUP);
  pinMode(WIO_KEY_C, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIO_KEY_A), clear_button_pressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(WIO_KEY_C), resync_button_pressed, FALLING);
 
  /* Setup our input GPIO. */
  Serial1.end(); // Disable hardware UART
  pinMode(PROBE_INPUT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PROBE_INPUT), track_state, CHANGE);

  /* Setup our buzzer. */
  pinMode(WIO_BUZZER, OUTPUT);

  /* Set default state. */
  mp_state_t g_state = WAITING;
  g_nb_measures = 0;

  /* Set our timer. */
  g_usec_counter = 0;

  /* Start timer (call u_sec_counter each microsecond. */
  ITimer0.attachInterruptInterval(1, u_sec_counter);

  /* Display wait screen. */
  wait_screen();
}


/**
 * @brief Try to decode a captured signal based on possible UART parameters.
 * @param period: period used to reconstruct the UART frame (number of samples per bit)
 * @param nbits: number of bits to code 1 'byte'
 * @param bit_duration: duration of 1 bit in microsecond (used to compute baudrate)
 * @param parity: parity (none, odd, even)
 * @param p_data: pointer to an output buffer
 * @param p_data_size: pointer to the size of the output buffer
 * @return number of encountered errors.
 */
 
int try_decode(int period, int nbits, int bit_duration, int parity, char *p_data, int *p_data_size)
{
  int i,j,bitpos,m,ones,errors;
  uint8_t state=1;
  uint8_t bits[10];
  unsigned char b;
  uint32_t tzero;
  bool b_parity_enabled = false;
  int nbytes;
  
  enum {
    START,
    DATA,
    PARITY,
    STOP
  };
  
  uint8_t decode_state = STOP;
    
  /* bit duration. */
  //const double t = 1000000.0/baudrate;
  const double t = bit_duration;
  double t_off = 0.0;
  tzero = g_measures[0]; /* timestamp du startbit, en usec */

  /* Determine parity bit size. */
  if (parity != PAR_NONE)
  {
    b_parity_enabled = true;
  }
  
  /* No errors encountered yet. */
  errors = 0;
  nbytes = 0;

  i=0;
  m=0;
  t_off = tzero;
  while (m < g_nb_measures)
  {
    /* track state change. */
    if (t_off >= g_measures[m])
    {
      /* Tuning. */
      tzero = g_measures[m];
      i = 0;
      
      //Serial.println("-- state change detected");
      state = (~state)&1;
      m++;
      
      /* Falling edge: start bit. */
      if ((decode_state == STOP) && !state)
      {
        //Serial.println("Falling edge, start bit detected.");
        decode_state = START;
      }
    }
    
    /* save bit. */
    if (t_off > (tzero + i*t + t/2) )
    {
      switch(decode_state)
      {
        case DATA:
          {
            bits[bitpos++] = state;

            if (bitpos >= (nbits + ((b_parity_enabled)?1:0)))
            {
              /* parity not supported yet, goto stop bit. */
              decode_state = STOP;

              /* Decode byte and display. */
              b=0;
              ones=0;
              for (j=0; j<nbits; j++)
              {
                /* Count 1's. */
                if (bits[j]&1)
                  ones++;

                /* Compute byte. */
                b |= ((bits[j]&1) << j);
              }

              /* Check parity if required. */
              if (b_parity_enabled)
              {
                switch(parity)
                {
                  case PAR_ODD:
                    {
                      if (bits[nbits] != ((ones%2)==0)?1:0)
                        errors++;
                    }
                    break;

                  case PAR_EVEN:
                    {
                      if (bits[nbits] != (ones%2))
                        errors++;
                    }
                    break;
                  
                  default:
                    break;
                }
              }

              /* Save decoded byte. */
              if (nbytes < *p_data_size)
              {
                p_data[nbytes++] = b;
              }
              else
              {
                /* Return the number of bytes written. */
                *p_data_size = nbytes;
              
              
                /* Return number of errors. */
                return errors;
              }
              
            }
          }
          break;
          
          
        case START:
          {
            decode_state = DATA;
            bitpos = 0;
          }
          break;

        default:
          break;
      }

      i++;
    }
    
    t_off += period;
  }

  /* Return the number of bytes written. */
  *p_data_size = nbytes;


  /* Return number of errors. */
  return errors;
}

/**
 * Main loop.
 */

void loop() {
  uint32_t min_inter, delta, i, total, nbits, bytesize;
  char rx_data[10];
  int bufsize;
  int parity, printable, found;
  int nbytes;
  char rx_buf[SCREEN_WIDTH];
  char rx_byte;

  switch(g_state)
  {
    case WAITING:
    case ACQUIRE:
    {
      /* Update nb measures. */
      update_waiting_measures();
    }
    break;
    
    case ANALYZE:
    {
      /* Determine bit duration. */
      min_inter = g_last_timestamp;
      
      /* Look for minimum interval (1 bit coded). */
      for (i=0; i<(g_nb_measures-1); i++)
      {
        delta = g_measures[i+1] - g_measures[i];
        if (delta < min_inter)
          min_inter = delta;
      }
  
      
      Serial.print("Bit duration: ");
      Serial.println(min_inter);
      
  
      /* Bruteforce parameters. */
      found = 0;
      for (bytesize=7; bytesize<9; bytesize++)
      {
        good_par = PAR_NONE;
        for (parity=PAR_NONE; parity<PAR_MAX; parity++)
        {
          bufsize = 10;
          if (try_decode(min_inter/4, bytesize, min_inter, parity, rx_data, &bufsize) == 0)
          {
            /* All characters are printable: found it ! */
            printable = count_printable_chars(rx_data, bufsize);
            if (printable == bufsize)
            {
              found = 1;
              
              /* Keep the best parity setting. */
              if (parity != PAR_NONE)
                good_par = parity;
            }
          }
        }
  
        /* Display parameters if found. */
        if (found)
        {
          g_bytesize = bytesize;
          
          Serial.print("Bit duration (us): ");
          Serial.println(min_inter);
          Serial.print("Byte size: ");
          Serial.println(bytesize);
          Serial.print("Parity: ");
          switch (good_par)
          {
            case PAR_NONE:
              Serial.println("none");
              uart_config = (bytesize - 5)*2;
              break;
  
            case PAR_EVEN:
              Serial.println("even");
              uart_config = (bytesize - 5)*2 + 0x20;
              break;
  
            case PAR_ODD:
              Serial.println("odd");
              uart_config = (bytesize - 5)*2 + 0x30;
              break;
          }
          baudrate = period_to_baudrate(min_inter);
          Serial.print("Baudrate: ");
          Serial.println(baudrate);
          Serial.print("UART config: ");
          Serial.println(uart_config, HEX);
        }
      }
  
      /* UART parameters recovered, configure hardware UART. */
      if ((found) && (baudrate > 0))
      {
        Serial.println("======< MONITOR >=============");
        
        /* Switch to display mode. */
        g_state = MONITOR;

        g_nb_measures = MAX_MEASURES;
        update_waiting_measures();

        /* Beeeeep. */
        playTone(1915, 100);

        /* Clear screen. */
        tft.fillRect(0, 0, 320, 240,BG_COLOR);

        /* Update status bar: display baudrate and config. */
        update_status_bar();
        clear_log_screen();

        /* Re-enable hardware UART. */
        detachInterrupt(digitalPinToInterrupt(PROBE_INPUT));
        Serial1.begin(baudrate, uart_config);
      }
      else
      {
        Serial.println("> Switch to acquisition mode");
        /* Back to waiting. */
        g_state = WAITING;
      }
    }
    break;

    case MONITOR:
    {
      /* Read data from UART. */
      nbytes = Serial1.available();
      if (nbytes > 0)
      {
        if (nbytes > SCREEN_WIDTH)
          nbytes = SCREEN_WIDTH;
        rx_byte = Serial1.readBytes(rx_buf, nbytes);
        /* Forward to Serial. */
        Serial.write(rx_buf, nbytes);

        /* Store data in memory. */
        for (i=0; i<nbytes; i++)
        {
          /* Jump to next line if '\n' received. */
          if (rx_buf[i] == '\n')
          {
            /* Update display. */
            tft.fillRect(0, g_uart_cur_line*LINE_HEIGHT + LOG_OFFSET_Y, 320, LINE_HEIGHT, BG_COLOR);
            tft.drawString(g_uart_log[g_uart_cur_line], 0, g_uart_cur_line*LINE_HEIGHT + LOG_OFFSET_Y);
            
            g_uart_cur_line = (g_uart_cur_line + 1) % SCREEN_HEIGHT;

            if (g_uart_cur_line == g_uart_top_line)
            {
              g_uart_top_line = (g_uart_top_line + 1) % SCREEN_HEIGHT;
              memset(&g_uart_log[g_uart_cur_line], 0, SCREEN_WIDTH+1);
            }
          }
          else if (rx_buf[i] == '\r')
          {
            g_uart_cur_col = 0;
          }
          else
          {
            if (g_uart_cur_col < SCREEN_WIDTH)
            {
              g_uart_log[g_uart_cur_line][g_uart_cur_col++] = rx_buf[i];
            }
          }
        }

        /* Update display. */
        tft.fillRect(0, g_uart_cur_line*LINE_HEIGHT + LOG_OFFSET_Y, 320, LINE_HEIGHT, TFT_RED);
        tft.drawString(g_uart_log[g_uart_cur_line], 0, g_uart_cur_line*LINE_HEIGHT + LOG_OFFSET_Y);
      }
    }
    break;

    default:
      break;
  }
}
