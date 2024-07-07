from machine import Pin, I2C
import time
from time import sleep_ms
import utime
import _thread

#STOPWATCH FUNCTIONS BEGIN
class stopwatch:
    IDLE = 1
    STARTED = 2
    toGREEN = 3
    GREEN = 4
    toYELLOW = 5
    YELLOW = 6
    toRED = 7
    RED = 8
    ms_per_minute = 60 * 1000
    def __init__(self):
        self.running = False
        self.state = stopwatch.IDLE
        self.start_ms = time.ticks_ms()
        self.last_ms = self.start_ms
        self.green_ms = 0
        self.yellow_ms = 0
        self.red_ms = 0
        
    def start(self, green_minutes, red_minutes):
        if self.running:
            stop()
        self.green_ms = green_minutes * stopwatch.ms_per_minute
        self.red_ms = red_minutes * stopwatch.ms_per_minute
        self.yellow_ms = self.green_ms + ((self.red_ms - self.green_ms) * 0.5)
        self.state = stopwatch.STARTED
        self.start_ms = time.ticks_ms()
        self.last_ms = self.start_ms
        self.running = True
    def elapsed(self):
        return self.last_ms - self.start_ms
    def tick(self):
        if self.running:
            self.last_ms = time.ticks_ms()
            if self.state == stopwatch.toGREEN:
                self.state = stopwatch.GREEN
            if self.state == stopwatch.toYELLOW:
                self.state = stopwatch.YELLOW
            if self.state == stopwatch.toRED:
                self.state = stopwatch.RED
            if self.state == stopwatch.STARTED:
                if self.elapsed() >= self.green_ms:
                    self.state = stopwatch.toGREEN
            if self.state == stopwatch.GREEN:
                if self.elapsed() >= self.yellow_ms:
                    self.state = stopwatch.toYELLOW
            if self.state == stopwatch.YELLOW:
                if self.elapsed() >= self.red_ms:
                    self.state = stopwatch.toRED
            
            
    def stop(self):
        self.running = False
        
    def mmss(self):
        seconds = (self.last_ms - self.start_ms) * 0.001
        minutes = 0
        while seconds >= 60:
            minutes = minutes + 1
            seconds = seconds - 60
        return minutes, seconds
    def state_str(self):
        if self.state == stopwatch.IDLE:
            return "IDLE"
        if self.state == stopwatch.STARTED:
            return "STARTED"
        if self.state == stopwatch.GREEN:
            return "GREEN"
        if self.state == stopwatch.YELLOW:
            return "YELLOW"
        if self.state == stopwatch.RED:
            return "RED"
        return "TRANSITION"
    def timer_as_str(self):
        mm = self.mmss()
        return f"{self.state_str()} == {mm[0]:0>2}:{mm[1]:0>2.0f}"
#STOPWATCH FUNCTIONS END

#KEYPAD FUNCTIONS BEGIN

#Setup the inputs and outputs being used
R1 = machine.Pin(22,machine.Pin.OUT)
R2 = machine.Pin(21,machine.Pin.OUT)
R3 = machine.Pin(10,machine.Pin.OUT)
R4 = machine.Pin(11,machine.Pin.OUT)
C1 = machine.Pin(12,machine.Pin.IN,machine.Pin.PULL_DOWN)
C2 = machine.Pin(13,machine.Pin.IN,machine.Pin.PULL_DOWN)
C3 = machine.Pin(15,machine.Pin.IN,machine.Pin.PULL_DOWN)
C4 = machine.Pin(14,machine.Pin.IN,machine.Pin.PULL_DOWN)

User_Key = "null" 			#User pressed key is set in the thread
                            #can be read in main loop

def Keyboard_Scanner():		#Function to handle the keyboard thread
    global User_Key
    Lock = "UNLOCKED"		#Variable ock is used to compare against
                            #action to occur
    while True:			#Loop forever
        Key_Pressed = "null"
        #Power each rown one by one.
        #While rowo is powered test each of the four columns
        #to see if any are  high. If button is pressed record
        R1.value(1)
        R2.value(0)
        R3.value(0)
        R4.value(0)
        if C1.value() == True: Key_Pressed = "1"
        if C2.value() == True: Key_Pressed = "2"
        if C3.value() == True: Key_Pressed = "3"
        if C4.value() == True: Key_Pressed = "A"
        #row 2
        R1.value(0)
        R2.value(1)
        R3.value(0)
        R4.value(0)
        if C1.value() == True: Key_Pressed = "4"
        if C2.value() == True: Key_Pressed = "5"
        if C3.value() == True: Key_Pressed = "6"
        if C4.value() == True: Key_Pressed = "B"
        #row 3
        R1.value(0)
        R2.value(0)
        R3.value(1)
        R4.value(0)
        if C1.value() == True: Key_Pressed = "7"
        if C2.value() == True: Key_Pressed = "8"
        if C3.value() == True: Key_Pressed = "9"
        if C4.value() == True: Key_Pressed = "C"
        #last lrow
        R1.value(0)
        R2.value(0)
        R3.value(0)
        R4.value(1)
        if C1.value() == True: Key_Pressed = "*"
        if C2.value() == True: Key_Pressed = "0"
        if C3.value() == True: Key_Pressed = "#"
        if C4.value() == True: Key_Pressed = "D"
        #key was not pressed during this pass and the lock is set
        #this will prevent repeat key strokes.
        #user must intentionaly depress key first
        if (Lock == "LOCKED") and (Key_Pressed == "null"):
            Lock = "UNLOCKED"
        #key ws pressed and since this is unlocked,
        #this is a new key press so we will update our global var
        if (Lock == "UNLOCKED") and (Key_Pressed != "null"):
            Lock = "LOCKED"
            User_Key = Key_Pressed            
        utime.sleep(.02)
            
#KEYBOARD FUNCTIONS END


#LCD FUNCTIONS BEGIN

"""Provides an API for talking to HD44780 compatible character LCDs."""


class LcdApi:
    """Implements the API for talking with HD44780 compatible character LCDs.
    This class only knows what commands to send to the LCD, and not how to get
    them to the LCD.

    It is expected that a derived class will implement the hal_xxx functions.
    """

    # The following constant names were lifted from the avrlib lcd.h
    # header file, however, I changed the definitions from bit numbers
    # to bit masks.
    #
    # HD44780 LCD controller command set

    LCD_CLR = 0x01              # DB0: clear display
    LCD_HOME = 0x02             # DB1: return to home position

    LCD_ENTRY_MODE = 0x04       # DB2: set entry mode
    LCD_ENTRY_INC = 0x02        # --DB1: increment
    LCD_ENTRY_SHIFT = 0x01      # --DB0: shift

    LCD_ON_CTRL = 0x08          # DB3: turn lcd/cursor on
    LCD_ON_DISPLAY = 0x04       # --DB2: turn display on
    LCD_ON_CURSOR = 0x02        # --DB1: turn cursor on
    LCD_ON_BLINK = 0x01         # --DB0: blinking cursor

    LCD_MOVE = 0x10             # DB4: move cursor/display
    LCD_MOVE_DISP = 0x08        # --DB3: move display (0-> move cursor)
    LCD_MOVE_RIGHT = 0x04       # --DB2: move right (0-> left)

    LCD_FUNCTION = 0x20         # DB5: function set
    LCD_FUNCTION_8BIT = 0x10    # --DB4: set 8BIT mode (0->4BIT mode)
    LCD_FUNCTION_2LINES = 0x08  # --DB3: two lines (0->one line)
    LCD_FUNCTION_10DOTS = 0x04  # --DB2: 5x10 font (0->5x7 font)
    LCD_FUNCTION_RESET = 0x30   # See "Initializing by Instruction" section

    LCD_CGRAM = 0x40            # DB6: set CG RAM address
    LCD_DDRAM = 0x80            # DB7: set DD RAM address

    LCD_RS_CMD = 0
    LCD_RS_DATA = 1

    LCD_RW_WRITE = 0
    LCD_RW_READ = 1

    def __init__(self, num_lines, num_columns):
        self.num_lines = num_lines
        if self.num_lines > 4:
            self.num_lines = 4
        self.num_columns = num_columns
        if self.num_columns > 40:
            self.num_columns = 40
        self.cursor_x = 0
        self.cursor_y = 0
        self.implied_newline = False
        self.backlight = True
        self.display_off()
        self.backlight_on()
        self.clear()
        self.hal_write_command(self.LCD_ENTRY_MODE | self.LCD_ENTRY_INC)
        self.hide_cursor()
        self.display_on()

    def clear(self):
        """Clears the LCD display and moves the cursor to the top left
        corner.
        """
        self.hal_write_command(self.LCD_CLR)
        self.hal_write_command(self.LCD_HOME)
        self.cursor_x = 0
        self.cursor_y = 0

    def show_cursor(self):
        """Causes the cursor to be made visible."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR)

    def hide_cursor(self):
        """Causes the cursor to be hidden."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def blink_cursor_on(self):
        """Turns on the cursor, and makes it blink."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR | self.LCD_ON_BLINK)

    def blink_cursor_off(self):
        """Turns on the cursor, and makes it no blink (i.e. be solid)."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY |
                               self.LCD_ON_CURSOR)

    def display_on(self):
        """Turns on (i.e. unblanks) the LCD."""
        self.hal_write_command(self.LCD_ON_CTRL | self.LCD_ON_DISPLAY)

    def display_off(self):
        """Turns off (i.e. blanks) the LCD."""
        self.hal_write_command(self.LCD_ON_CTRL)

    def backlight_on(self):
        """Turns the backlight on.

        This isn't really an LCD command, but some modules have backlight
        controls, so this allows the hal to pass through the command.
        """
        self.backlight = True
        self.hal_backlight_on()

    def backlight_off(self):
        """Turns the backlight off.

        This isn't really an LCD command, but some modules have backlight
        controls, so this allows the hal to pass through the command.
        """
        self.backlight = False
        self.hal_backlight_off()

    def move_to(self, cursor_x, cursor_y):
        """Moves the cursor position to the indicated position. The cursor
        position is zero based (i.e. cursor_x == 0 indicates first column).
        """
        self.cursor_x = cursor_x
        self.cursor_y = cursor_y
        addr = cursor_x & 0x3f
        if cursor_y & 1:
            addr += 0x40    # Lines 1 & 3 add 0x40
        if cursor_y & 2:    # Lines 2 & 3 add number of columns
            addr += self.num_columns
        self.hal_write_command(self.LCD_DDRAM | addr)

    def putchar(self, char):
        """Writes the indicated character to the LCD at the current cursor
        position, and advances the cursor by one position.
        """
        if char == '\n':
            if self.implied_newline:
                # self.implied_newline means we advanced due to a wraparound,
                # so if we get a newline right after that we ignore it.
                pass
            else:
                self.cursor_x = self.num_columns
        else:
            self.hal_write_data(ord(char))
            self.cursor_x += 1
        if self.cursor_x >= self.num_columns:
            self.cursor_x = 0
            self.cursor_y += 1
            self.implied_newline = (char != '\n')
        if self.cursor_y >= self.num_lines:
            self.cursor_y = 0
        self.move_to(self.cursor_x, self.cursor_y)

    def putstr(self, string):
        """Write the indicated string to the LCD at the current cursor
        position and advances the cursor position appropriately.
        """
        for char in string:
            self.putchar(char)

    def custom_char(self, location, charmap):
        """Write a character to one of the 8 CGRAM locations, available
        as chr(0) through chr(7).
        """
        location &= 0x7
        self.hal_write_command(self.LCD_CGRAM | (location << 3))
        self.hal_sleep_us(40)
        for i in range(8):
            self.hal_write_data(charmap[i])
            self.hal_sleep_us(40)
        self.move_to(self.cursor_x, self.cursor_y)

    def hal_backlight_on(self):
        """Allows the hal layer to turn the backlight on.

        If desired, a derived HAL class will implement this function.
        """
        pass

    def hal_backlight_off(self):
        """Allows the hal layer to turn the backlight off.

        If desired, a derived HAL class will implement this function.
        """
        pass

    def hal_write_command(self, cmd):
        """Write a command to the LCD.

        It is expected that a derived HAL class will implement this
        function.
        """
        raise NotImplementedError

    def hal_write_data(self, data):
        """Write data to the LCD.

        It is expected that a derived HAL class will implement this
        function.
        """
        raise NotImplementedError

    def hal_sleep_us(self, usecs):
        """Sleep for some time (given in microseconds)."""
        time.sleep_us(usecs)

"""Implements a HD44780 character LCD connected via PCF8574 on I2C.
   This was tested with: https://www.wemos.cc/product/d1-mini.html"""



# The PCF8574 has a jumper selectable address: 0x20 - 0x27
DEFAULT_I2C_ADDR = 0x27

# Defines shifts or masks for the various LCD line attached to the PCF8574

MASK_RS = 0x01
MASK_RW = 0x02
MASK_E = 0x04
SHIFT_BACKLIGHT = 3
SHIFT_DATA = 4


class I2cLcd(LcdApi):
    """Implements a HD44780 character LCD connected via PCF8574 on I2C."""

    def __init__(self, i2c, i2c_addr, num_lines, num_columns):
        self.i2c = i2c
        self.i2c_addr = i2c_addr
        self.i2c.writeto(self.i2c_addr, bytearray([0]))
        sleep_ms(20)   # Allow LCD time to powerup
        # Send reset 3 times
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        sleep_ms(5)    # need to delay at least 4.1 msec
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        sleep_ms(1)
        self.hal_write_init_nibble(self.LCD_FUNCTION_RESET)
        sleep_ms(1)
        # Put LCD into 4 bit mode
        self.hal_write_init_nibble(self.LCD_FUNCTION)
        sleep_ms(1)
        LcdApi.__init__(self, num_lines, num_columns)
        cmd = self.LCD_FUNCTION
        if num_lines > 1:
            cmd |= self.LCD_FUNCTION_2LINES
        self.hal_write_command(cmd)

    def hal_write_init_nibble(self, nibble):
        """Writes an initialization nibble to the LCD.

        This particular function is only used during initialization.
        """
        byte = ((nibble >> 4) & 0x0f) << SHIFT_DATA
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))

    def hal_backlight_on(self):
        """Allows the hal layer to turn the backlight on."""
        self.i2c.writeto(self.i2c_addr, bytearray([1 << SHIFT_BACKLIGHT]))

    def hal_backlight_off(self):
        """Allows the hal layer to turn the backlight off."""
        self.i2c.writeto(self.i2c_addr, bytearray([0]))

    def hal_write_command(self, cmd):
        """Writes a command to the LCD.

        Data is latched on the falling edge of E.
        """
        byte = ((self.backlight << SHIFT_BACKLIGHT) | (((cmd >> 4) & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))
        byte = ((self.backlight << SHIFT_BACKLIGHT) | ((cmd & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))
        if cmd <= 3:
            # The home and clear commands require a worst case delay of 4.1 msec
            sleep_ms(5)

    def hal_write_data(self, data):
        """Write data to the LCD."""
        byte = (MASK_RS | (self.backlight << SHIFT_BACKLIGHT) | (((data >> 4) & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))
        byte = (MASK_RS | (self.backlight << SHIFT_BACKLIGHT) | ((data & 0x0f) << SHIFT_DATA))
        self.i2c.writeto(self.i2c_addr, bytearray([byte | MASK_E]))
        self.i2c.writeto(self.i2c_addr, bytearray([byte]))

#LCD FUNCTIONS END


#Normal program begins

#initializei the stopwatch so we can do timers
myTimer = stopwatch()

#Print Status function
last_line1 = ""
last_line2 = ""

def Print_Status():
    global last_line1
    global last_line2
    line1 = "Speech Timer    "
    line2 = myTimer.timer_as_str()
    if (line1 != last_line1) or (line2 != last_line2):
        lcd.clear()
        lcd.putstr(line1)
        lcd.putstr(line2)
        last_line1 = line1
        last_line2 = line2
    
#This runs the keyboard in a continual separate thread.
_thread.start_new_thread(Keyboard_Scanner,())	        

#Initialize the LCD Display
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)
addr = i2c.scan()
      
lcd = I2cLcd(i2c, addr[0], 2, 16)

myTimer.start(5, 7)
last_time = 0
while True:
    myTimer.tick()
   
    if myTimer.state == stopwatch.toGREEN:
        print("I Caught the green transition. i can turn onlinghts")
    if myTimer.state == stopwatch.toYELLOW:
        print("I caught the yellow transition")
    if myTimer.state == stopwatch.toRED:
        print("I caught the red transition. CUT IT OUT")
    
    #check for keypad
    if User_Key != "null":
        Key_Code = User_Key
        User_Key = "null"
        print("Key Code =",Key_Code)
    Print_Status()
    sleep_ms(100)
