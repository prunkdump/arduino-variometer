#include <IntTW.h>

#include <Arduino.h>
#include <avr/pgmspace.h>

#include <VarioSettings.h>

#define COMMON_TWCR_FLAGS _BV(TWINT) | _BV(TWEN) | _BV(TWIE)

#define TW_WRITE 0x00
#define TW_READ 0x01
#define TW_START 0x08
#define TW_REP_START 0x10
#define TW_MT_SLA_ACK 0x18
#define TW_MT_DATA_ACK 0x28
#define TW_MR_SLA_ACK 0x40
#define TW_MR_DATA_ACK 0x50
#define TW_MR_DATA_NACK 0x58

/* never returned by the chip */
/* used only on the code */
#define TW_STOP 0x59
	  
IntTW intTW;

/* manage two wires interrupts */
ISR(TWI_vect) {
  intTW.twiVect();
}
  
void IntTW::begin(bool internalPullUp) {

  /* set pins */
  if( internalPullUp ) {
    digitalWrite(SDA, 1);
    digitalWrite(SCL, 1);
  }

  /* set prescaler */
  TWSR &= 0xFC;
  TWBR = ((F_CPU/VARIO_TW_FREQ) - 16)/2;
}

void IntTW::setTxBuffer(uint8_t* buff) {

  txBuffer = buff;
}

void IntTW::setRxBuffer(uint8_t* buff) {

  rxBuffer = buff;
}

void IntTW::start(uint8_t* commands, uint8_t commandLength, uint8_t commandFlags, void (*successCallback)(void)) {

  /* save callback, can be NULL */
  callback = successCallback;

  /* save previous bus state */
  uint8_t haveBus = flags & INTTW_KEEP_BUS;
  flags = commandFlags;

  /* save command buffer */
  cmd = commands;
  cmdStart = commands;
  cmdLength = commandLength;

  /************************/
  /* check if we have bus */
  /************************/

  /* keeping bus to read to the same device is not allowed */
  /* check the write case */
  bool continueToWrite = false;
  if( haveBus && canContinue(cmd) ) {
    continueToWrite = true;
  }

  /* update the action */
  parseAction();
  if( continueToWrite ){
    transmitByte(); //this update TWCR and expectedState
  } else {
    /* send start or rep start */ 
    TWCR = _BV(TWSTA) | COMMON_TWCR_FLAGS;
    if( haveBus ) {
      expectedState = TW_REP_START;
    } else {
      expectedState = TW_START;
    }
  }
}
  

bool IntTW::transmitting(void) {

  /* on error, no transmitting */
  if( !cmd ) {
    return false;
  }
  
  /* if no error */
  else {

    /* wait for the stop command */
    if( expectedState != TW_STOP ) {
      return true;
    }

    /* if stop launched, wait for flag cleared */
    /* (when keeping bus this flag is not set) */
    else {
      if( TWCR & _BV(TWSTO) ) {
	return true;
      }
    }
  }
      
  return false;
}


bool IntTW::succeeded(void) {

  if( !transmitting() && cmd ) {
    return true;
  }

  return false;
}

	
/* called when the previous command is done */
/* and when we can't continue */
void IntTW::checkStopOrRestart(void) {

  /* if not finished */
  if( cmd - cmdStart < cmdLength ) {

    /* send repeated start */
    parseAction();
    TWCR = _BV(TWSTA) | COMMON_TWCR_FLAGS;
    expectedState = TW_REP_START;
    
  } else {
    /**************/
    /* finished ! */
    /**************/
    expectedState = TW_STOP; //signal command end
    
    /* need to keed bus ? */
    if( flags & INTTW_KEEP_BUS ) {
      TWCR = _BV(TWEN); //disable twi interrupts but don't clear flag
    } else {
      TWCR = _BV(TWSTO) | COMMON_TWCR_FLAGS; //stop
    }
    /* need to callback ? */
    if( callback ) {
      callback();
    }
  }
}


/* check if we can continue from the current command to the next command */
bool IntTW::canContinue(uint8_t* nextCmdPtr) {

  bool canContinue = false;
  
  /* check if there is another cmd */
  if( nextCmdPtr - cmdStart < cmdLength ) {

    /* get next address */
    uint8_t nextAddress;
    if( flags & INTTW_USE_PROGMEM ) {
      nextAddress = pgm_read_byte_near(nextCmdPtr);
    } else {
      nextAddress = nextCmdPtr[0];
    }

    /* check next address */
    if( nextAddress == currentAddress ) {
      canContinue = true;
    }
  }

  return canContinue;
}

/* must be run only when command finished and keeping bus */			
void IntTW::stop(void) {
  
  expectedState = TW_STOP; //signal command end
  flags = 0; //we release the bus
  TWCR = _BV(TWSTO) | COMMON_TWCR_FLAGS;
}


/* parse address, count (and pointer if needed) */
/* update current command variables */
void IntTW::parseAction(void) {

  /* read address and count */
  if( flags & INTTW_USE_PROGMEM ) {
    currentAddress = pgm_read_byte_near(cmd);
    count = pgm_read_byte_near(cmd + 1);
  } else {
    currentAddress = cmd[0];
    count = cmd[1];
  }
  cmd += 2;

#ifdef INTTW_LOAD_POINTER_FUNC
  /* if needed read pointer */
  if( (count & INTTW_COUNT_FLAG_MASK) == INTTW_SET_POINTER ) {
    
    uint8_t* newPointer;
    if( flags & INTTW_USE_PROGMEM ) {
      newPointer = (uint8_t*)pgm_read_word(cmd);
    } else {
      newPointer = ((uint8_t**)cmd)[0];
    }
    cmd += sizeof(uint8_t*);
    
    if( currentAddress & TW_READ ) {
      rxBuffer = newPointer;
    } else {
      txBuffer = newPointer;
    }
  }
#endif
}


/* send a byte and update TWCR and expectedState */
void IntTW::transmitByte(void) {
  
  /* check where to read the TX byte */
  if( count & INTTW_COUNT_FLAG_USE_POINTER ) {
    TWDR = *txBuffer;
    txBuffer++;
  } else {
    if( flags & INTTW_USE_PROGMEM ) {
      TWDR = pgm_read_byte_near(cmd);
    } else {
      TWDR = *cmd;
    }
    cmd++;
  }

  /* send */
  count--;
  TWCR = COMMON_TWCR_FLAGS;
  expectedState = TW_MT_DATA_ACK;
}
  
  

/************************/
/* the main TW function */
/************************/
void IntTW::twiVect(void) {

  /* get state without prescaler */
  uint8_t twState = TWSR & 0xF8;

  /* stop on error */
  if( !cmd || ( twState != expectedState ) ) {
    //expectedState = TW_STO is not needed on error
    cmd = 0; //signal error
    flags = 0; //we release the bus
    TWCR = _BV(TWSTO) | COMMON_TWCR_FLAGS;

#ifdef INTTW_CALLBACK_ON_ERROR_FUNC
    /* launch callback on error */
    if( callback ) {
      callback();
    }
#endif

    return;
  }

  /*********/
  /* CASES */
  /*********/
  
  /* transmitted start or restart */
  /*------------------------------*/
  if( twState == TW_START || twState == TW_REP_START ) {

    /* command is already parsed */
    /* just send the SLA+(R/W) */
    TWDR = currentAddress;
    TWCR = COMMON_TWCR_FLAGS;
    if( currentAddress & TW_READ ) {
      expectedState = TW_MR_SLA_ACK;
    } else {
      expectedState = TW_MT_SLA_ACK;
    }
  }
  

  /* we are Master Transmitter   */
  /* need to transmit one byte ? */
  /*-----------------------------*/
  else if(twState == TW_MT_SLA_ACK || twState == TW_MT_DATA_ACK) {

    /* if count == 0 we need to check if we can continue */
    /* if we can, we update count now to send the byte immediately */
    if( ((count & INTTW_COUNT_MASK) == 0) && canContinue(cmd) ) {
      parseAction();
    }      

    /* check (updated) count for remaining tx bytes */
    if( (count & INTTW_COUNT_MASK) != 0) {
      transmitByte();
    } else {
      checkStopOrRestart(); //we can't continue
    }
  }
  
  /* receive case  */
  /*---------------*/
  else {

    /* need to save the received byte */
    if( twState == TW_MR_DATA_ACK || twState == TW_MR_DATA_NACK ) {

      /* check where to store */
      if( count & INTTW_COUNT_FLAG_USE_POINTER ) {
	*rxBuffer = TWDR;
	rxBuffer++;
      } else {
	*cmd = TWDR;
	cmd++;
      }
      
      /* decrease count */
      count--; 
    }

    /* check if we need to ACK */
    if( twState == TW_MR_SLA_ACK || twState == TW_MR_DATA_ACK ) {

      /*************************************/
      /* if more than 1 byte remain -> ACK */
      /*************************************/
      bool needAck = false;

      /* count == 0 with ACK has a special mean */
      /* This mean than we need to continue to the next command */
      /* and update the count now */
      if( (count & INTTW_COUNT_MASK) == 0 ) {
	parseAction();
      }

      /* check if the current command is sufficient */
      if( (count & INTTW_COUNT_MASK) > 1 ) {
	needAck = true;
      }

      /* count==0 is not possible here */
      /* if count==1 check if more reads on the next command */
      else { //count = 1

	/* get next cmd */
	uint8_t* nextCmdPtr;
	if( count & INTTW_COUNT_FLAG_USE_POINTER ) {
	  nextCmdPtr = cmd;
	} else {
	  nextCmdPtr = cmd + 1;
	}

	/* check if we can continue */
	if( canContinue(nextCmdPtr) ) {
	  needAck = true;
	}
      }

      /* set ACK/NACK */
      if( needAck ) {
	TWCR = _BV(TWEA) | COMMON_TWCR_FLAGS;
	expectedState = TW_MR_DATA_ACK;	
      } else {
	TWCR = COMMON_TWCR_FLAGS;
	expectedState = TW_MR_DATA_NACK;
      }
    }

    /* in ALL other cases stop or restart */
    else { 
      checkStopOrRestart();
    }
  }
}


/**********************/
/* blocking functions */
/**********************/
bool IntTW::writeBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff) {
  
  uint8_t buildCommand[] = { INTTW_ACTION(address, INTTW_WRITE),
			     INTTW_DEST(1, INTTW_IN_CMD),
			     cmd,
			     INTTW_ACTION(address, INTTW_WRITE),
			     INTTW_DEST(count, INTTW_AT_POINTER) };

  /* check count */
  uint8_t cmdLength = sizeof(buildCommand);
  if( ! count )
    cmdLength -= 2; //remove second write
  
  /* launch command */
  setTxBuffer(buff);
  start(buildCommand, cmdLength, INTTW_NONE);
  while( transmitting() ) { }

  return succeeded();
}


bool IntTW::readBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff) {

  uint8_t buildCommand[] = { INTTW_ACTION(address, INTTW_WRITE),
			     INTTW_DEST(1, INTTW_IN_CMD),
			     cmd,
			     INTTW_ACTION(address, INTTW_READ),
			     INTTW_DEST(count, INTTW_AT_POINTER) };

  /* check count */
  uint8_t cmdLength = sizeof(buildCommand);
  if( ! count )
    cmdLength -= 2; //remove read
  
  
  /* launch command */
  setRxBuffer(buff);
  start(buildCommand, cmdLength, INTTW_NONE);
  while( transmitting() ) { }
  
  return succeeded();
}
