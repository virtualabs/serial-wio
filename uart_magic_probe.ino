#include "SAMDTimerInterrupt.h"

#define MAX_MEASURES    25600
#define IDLE_INTERVAL   100000

typedef enum {
  WAITING,
  ACQUIRE,
  ANALYZE
} mp_state_t;

enum {
  PAR_NONE = 0,
  PAR_ODD,
  PAR_EVEN,
  PAR_MAX
};

static uint32_t g_measures[MAX_MEASURES];
volatile uint32_t g_usec_counter;
volatile uint32_t g_last_timestamp;
volatile int g_nb_measures;
volatile mp_state_t g_state;

SAMDTimer ITimer0(TIMER_TC3);

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

void setup() {
  /* Setup WioTerminal Serial. */
  Serial.begin(115200);

  /* Setup our input GPIO. */
  pinMode(D3, INPUT);
  attachInterrupt(D3, track_state, CHANGE);

  /* Set default state. */
  mp_state_t g_state = WAITING;
  g_nb_measures = 0;

  /* Set our timer. */
  g_usec_counter = 0;

  /* Start timer (call u_sec_counter each microsecond. */
  ITimer0.attachInterruptInterval(1, u_sec_counter);
}

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

int try_decode(int nbits, int bit_duration, int parity, char *p_data, int *p_data_size)
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
  
  /*
  Serial.println("-----------------------------");
  Serial.print("measures: ");
  Serial.println(g_nb_measures);
  for (i=0; i< g_nb_measures; i++)
  {
    Serial.print(i);
    Serial.print(" - ");
    Serial.println(g_measures[i]);
  }
  Serial.println("-----------------------------");
  */
  
  /*
  Serial.print("Bit duration: ");
  Serial.println(t);
  Serial.print("T0: ");
  Serial.println(tzero);
  */

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
      
      //Serial.print("state changed: ");
      //Serial.println(state);

      /* Falling edge: start bit. */
      if ((decode_state == STOP) && !state)
      {
        //Serial.println("Falling edge, start bit detected.");
        decode_state = START;
      }
        
      /*
      Serial.print("measure index: ");
      Serial.print(m);
      Serial.print(" state: ");
      Serial.print(state);
      Serial.print(" (");
      Serial.print(t_off);
      Serial.println(")");
      */
    }
    
    /* save bit. */
    if (t_off > (tzero + i*t + t/2) )
    {
      switch(decode_state)
      {
        case DATA:
          {
            /*
            Serial.print("d[");
            Serial.print(bitpos);
            Serial.print("]=");
            Serial.println(state);
            */
            bits[bitpos++] = state;

            if (bitpos >= (nbits + ((b_parity_enabled)?1:0)))
            {
              //Serial.println("Expected bits collected");
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
                      {
                        //Serial.println("Parity error (odd but even number of 1s)");
                        errors++;
                      }
                    }
                    break;

                  case PAR_EVEN:
                    {
                      if (bits[nbits] != (ones%2))
                      {
                        errors++;
                        //Serial.println("Parity error");
                      }
                    }
                    break;
                  
                  default:
                    break;
                }
              }

              /* Display decoded byte. */
              //Serial.print((char)b);
              if (nbytes < *p_data_size)
                p_data[nbytes++] = b;
              
            }
          }
          break;
          
          
        case START:
          {
            //Serial.println("Start bit sampled, now collecting d0->dn");
            decode_state = DATA;
            bitpos = 0;
          }
          break;

        default:
          break;
      }

      i++;
      
      //Serial.print("-- t/2 reached, save bit (");
      //bits[i++] = state;
      /*
      Serial.print(state);
      Serial.println(")");
      */
    }
    
    t_off += 0.1;
  }

  /* Return the number of bytes written. */
  *p_data_size = nbytes;


  /* Return number of errors. */
  return errors;
}


void loop() {
  int baudrate = 0;
  uint32_t min_inter, delta, i, total, nbits;
  char rx_data[10];
  int bufsize;
  int bytesize, parity, printable, good_par, found;
  
  if (g_state == ANALYZE)
  {
    /* Determine bit duration. */
    min_inter = g_last_timestamp;
    
    /* Look for minimum interval (1 bit coded). */
    for (i=0; i<(g_nb_measures-1); i++)
    {
      delta = g_measures[i+1] - g_measures[i];
      //Serial.print("delta: ");
      //Serial.println(delta);
      if (delta < min_inter)
        min_inter = delta;
    }

    /*
    Serial.print("Bit duration: ");
    Serial.println(min_inter);
    */

    /* Bruteforce parameters. */
    found = 0;
    for (bytesize=7; bytesize<9; bytesize++)
    {
      good_par = PAR_NONE;
      for (parity=PAR_NONE; parity<PAR_MAX; parity++)
      {
        bufsize = 10;
        if (try_decode(bytesize, min_inter, parity, rx_data, &bufsize) == 0)
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
        Serial.print("Bit duration (us): ");
        Serial.println(min_inter);
        Serial.print("Byte size: ");
        Serial.println(bytesize);
        Serial.print("Parity: ");
        switch (good_par)
        {
          case PAR_NONE:
            Serial.println("none");
            break;

          case PAR_EVEN:
            Serial.println("even");
            break;

          case PAR_ODD:
            Serial.println("odd");
            break;
        }

        /* Done. */
        break;
      }
    }
    
    
    #if 0
    min_inter = g_last_timestamp;
    
    /* Look for minimum interval (1 bit coded). */
    for (i=0; i<(g_nb_measures-1); i++)
    {
      delta = g_measures[i+1] - g_measures[i];
      //Serial.print("delta: ");
      //Serial.println(delta);
      if (delta < min_inter)
        min_inter = delta;
    }

    /* Display min_inter. */
    Serial.print("min inter: ");
    Serial.println(min_inter);

    /* Compute acquisition time. */
    total = 0;
    for (i=0; i<(g_nb_measures-1); i++)
    {
      delta = g_measures[i+1] - g_measures[i];
      if (delta < 10*min_inter)
      {
        total += delta;
      }
    }

    Serial.print("total: ");
    Serial.println(total);

    /* Deduce number of bits. */
    nbits = total / min_inter;
    Serial.print("bits: ");
    Serial.println(nbits);
    #endif

    /* Back to waiting. */
    g_state = WAITING;
  }
}

void track_state(void)
{
  switch(g_state)
  {
    case WAITING:
      {
        /* Found a falling edge, start acquisition. */
        if (digitalRead(D3) == 0)
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
          /* Switch to analyze mode. */
          g_state = ANALYZE;
        }
      }
      break;
  }
}
