--+----------------------------------------------------------------------------
--| 
--| COPYRIGHT 2018 United States Air Force Academy All rights reserved.
--| 
--| United States Air Force Academy     __  _______ ___    _________ 
--| Dept of Electrical &               / / / / ___//   |  / ____/   |
--| Computer Engineering              / / / /\__ \/ /| | / /_  / /| |
--| 2354 Fairchild Drive Ste 2F6     / /_/ /___/ / ___ |/ __/ / ___ |
--| USAF Academy, CO 80840           \____//____/_/  |_/_/   /_/  |_|
--| 
--| ---------------------------------------------------------------------------
--|
--| FILENAME      : top_basys3.vhd
--| AUTHOR(S)     : Capt Dan Johnson
--| CREATED       : 4/22/2020
--| DESCRIPTION   : This file implements the top level module solution for a BASYS 3 to 
--|					drive the Lab 4 Design Project (Advanced Elevator Controller).
--|
--|					Inputs: clk       --> 100 MHz clock from FPGA
--|							btnL      --> Rst Clk
--|							btnR      --> Rst FSM
--|							btnU      --> Rst Master
--|							btnC      --> GO (request floor)
--|							sw(15:12) --> Passenger location (floor select bits)
--| 						sw(3:0)   --> Desired location (floor select bits)
--| 						 - Minumum FUNCTIONALITY ONLY: sw(1) --> up_down, sw(0) --> stop
--|							 
--|					Outputs: led --> indicates elevator movement with sweeping pattern (additional functionality)
--|							   - led(10) --> led(15) = MOVING UP
--|							   - led(5)  --> led(0)  = MOVING DOWN
--|							   - ALL OFF		     = NOT MOVING
--|							 an(3:0)    --> seven-segment display anode active-low enable (AN3 ... AN0)
--|							 seg(6:0)	--> seven-segment display cathodes (CG ... CA.  DP unused)
--|
--| DOCUMENTATION : None
--|
--+----------------------------------------------------------------------------
--|
--| REQUIRED FILES :
--|
--|    Libraries : ieee
--|    Packages  : std_logic_1164, numeric_std
--|    Files     : MooreElevatorController_Concurrent.vhd, clock_divider.vhd, sevenSegDecoder.vhd
--|				   thunderbird_fsm_binary.vhd, sevenSegDecoder, TDM4.vhd, destinationController,
--|				   hexDecimalDecoder.vhd, 
--|
--+----------------------------------------------------------------------------
--|
--| NAMING CONVENSIONS :
--|
--|    xb_<port name>           = off-chip bidirectional port ( _pads file )
--|    xi_<port name>           = off-chip input port         ( _pads file )
--|    xo_<port name>           = off-chip output port        ( _pads file )
--|    b_<port name>            = on-chip bidirectional port
--|    i_<port name>            = on-chip input port
--|    o_<port name>            = on-chip output port
--|    c_<signal name>          = combinatorial signal
--|    f_<signal name>          = synchronous signal
--|    ff_<signal name>         = pipeline stage (ff_, fff_, etc.)
--|    <signal name>_n          = active low signal
--|    w_<signal name>          = top level wiring signal
--|    g_<generic name>         = generic
--|    k_<constant name>        = constant
--|    v_<variable name>        = variable
--|    sm_<state machine type>  = state machine type definition
--|    s_<signal name>          = state name
--|
--+----------------------------------------------------------------------------
library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;


entity top_basys3 is
	port(

		clk     :   in std_logic; -- native 100MHz FPGA clock
		
		-- Switches (16 total)
		sw  	:   in std_logic_vector(15 downto 0);
		
		-- Buttons (5 total)
		btnC	:	in	std_logic;					  -- GO
		btnU	:	in	std_logic;					  -- master_reset
		btnL	:	in	std_logic;                    -- clk_reset
		btnR	:	in	std_logic;	                  -- fsm_reset
		--btnD	:	in	std_logic;			
		
		-- LEDs (16 total)
		led 	:   out std_logic_vector(15 downto 0);

		-- 7-segment display segments (active-low cathodes)
		seg		:	out std_logic_vector(6 downto 0);

		-- 7-segment display active-low enables (anodes)
		an      :	out std_logic_vector(3 downto 0)
	);
end top_basys3;

architecture top_basys3_arch of top_basys3 is 
  
    component thunderbird_fsm_binary is 
    port(
	   i_clk, i_reset	:	in	std_logic;
	   i_left, i_right	:	in 	std_logic;
	   o_lights_L		:	out std_logic_vector(2 downto 0); -- LC downto LA
	   o_lights_R		:	out std_logic_vector(2 downto 0)  -- RC downto RA
     );
    end component;
    
    component sevenSegDecoder is 
      port(
        i_D  : in  std_logic_vector(3 downto 0);
        o_S  : out std_logic_vector(6 downto 0)
      );
    end component;
    
    component MooreElevatorController_Concurrent is
        Port ( i_clk        : in  STD_LOGIC;
               i_reset      : in  STD_LOGIC;
               i_stop       : in  STD_LOGIC;
               i_up_down    : in  STD_LOGIC;
               o_floor      : out STD_LOGIC_VECTOR (3 downto 0)
              );
    end component;
    
    component TDM4 is
        generic ( constant k_WIDTH : natural  := 4); -- bits in input and output
        Port ( i_CLK        : in  STD_LOGIC;
               i_RESET      : in  STD_LOGIC; -- asynchronous
               i_D3         : in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
               i_D2         : in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
               i_D1         : in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
               i_D0         : in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
               o_DATA       : out STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
               o_SEL        : out STD_LOGIC_VECTOR (3 downto 0)    -- selected data line (one-cold)
        );
    end component;
    
    component hexDecimalDecoder is 
      port(
        i_hex     : in std_logic_vector(3 downto 0);
        o_upper   : out std_logic_vector(3 downto 0);
        o_lower   : out std_logic_vector(3 downto 0)
      );
    end component;
    
    component clock_divider is
        generic ( constant k_DIV : natural := 2    ); -- How many clk cycles until slow clock toggles
                                                   -- Effectively, you divide the clk double this 
                                                   -- number (e.g., k_DIV := 2 --> clock divider of 4)
        port (  i_clk    : in std_logic;
                i_reset  : in std_logic;           -- asynchronous
                o_clk    : out std_logic           -- divided (slow) clock
        );
    end component;
    
    component destinationController is 
      port(
        i_currentFloor :   in std_logic_vector(3 downto 0);
        i_dest  :   in std_logic_vector(3 downto 0);
        o_up_down : out std_logic;
        o_stop    : out std_logic
      );
    end component;

    component pickupDropoff is 
    port(
	   i_go			: in std_logic;
	   i_clk			: in std_logic;
	   i_reset        : in std_logic;
	   i_passFloor 	: in std_logic_vector(3 downto 0);
	   i_destFloor    : in std_logic_vector(3 downto 0);
	   i_currentFloor : in std_logic_vector(3 downto 0);
	   o_dest 		: out std_logic_vector(3 downto 0)
    );
    end component;

  signal w_up_down, w_stop, c_left, c_right, w_clk_elevator, w_clk_tdm, w_clk_thunderbird, c_fsmReset, c_clkReset, c_passFloorZeroCheck, c_destFloorZeroCheck: std_logic;
  signal w_floor, w_upper, w_lower, w_dest, w_destFinal, f_destFloor, c_destFloor_next, f_passFloor, c_passFloor_next : std_logic_vector(3 downto 0);
  signal w_decUpper, w_decLower : std_logic_vector(6 downto 0);
  signal w_lights_L, w_lights_R : std_logic_vector(2 downto 0);
  
begin
	-- PORT MAPS ----------------------------------------

	hexDecmialDecoder : hexDecimalDecoder port map (
	  i_hex   => w_floor,
	  o_upper => w_upper,
	  o_lower => w_lower
	);
	
	TDM : TDM4 	
	generic map ( k_WIDTH => 7 )
    port map ( i_CLK   => w_clk_tdm,
               i_RESET => btnU,
               i_D3    => w_decUpper,
               i_D2    => w_decLower,
               i_D1    => "1111111",
               i_D0    => "1111111",
               o_DATA  => seg,
               o_SEL   => an
    );
	
	clk_divider_elevator : clock_divider 
    generic map ( k_DIV => 25000000 )
    port map (
        i_clk    => clk,
        i_reset  => c_clkReset,
        o_clk    => w_clk_elevator
    );
    
    clk_divider_tdm : clock_divider 
    generic map ( k_DIV => 100000 )
    port map (
        i_clk    => clk,
        i_reset  => btnU,
        o_clk    => w_clk_tdm
    );
    
    clk_divider_thunderbird : clock_divider 
    generic map ( k_DIV => 6250000 )
    port map (
        i_clk    => clk,
        i_reset  => c_clkReset,
        o_clk    => w_clk_thunderbird
    );
    
    sevenSegDecoder_upper : sevenSegDecoder port map (
        i_D    => w_upper, --w_upper
        o_S    => w_decUpper
    );
    
    sevenSegDecoder_lower : sevenSegDecoder port map (
        i_D    => w_lower, --w_lower
        o_S    => w_decLower
    );
    
    thunderbird_fsm_binar_inst : thunderbird_fsm_binary
        port map (
            i_clk        => w_clk_thunderbird,
            i_reset      => c_fsmReset,
            i_left       => c_left,
            i_right      => c_right,
            o_lights_L   => w_lights_L,
            o_lights_R   => w_lights_R
        );
        
	MooreElevatorController : MooreElevatorController_Concurrent port map (
            i_clk     => w_clk_elevator,
            i_reset   => c_fsmReset,
            i_stop    => w_stop,
            i_up_down => w_up_down,
            o_floor   => w_floor
        );
        
	destinationController_inst : destinationController port map (
          i_currentFloor => w_floor,
          i_dest		 => w_destFinal,
          o_up_down		 => w_up_down,
          o_stop		 => w_stop
        );

	pickup_dropoff_inst : pickupDropoff port map (
        i_go          => btnC,
        i_clk         => w_clk_elevator,
        i_reset       => c_fsmReset,
        i_passFloor   => f_passFloor,
        i_currentFloor=> w_floor,
        i_destFloor   => f_destFloor,
        o_dest        => w_destFinal
        );
               
	-- CONCURRENT STATEMENTS ----------------------------
	c_left <= w_up_down and not w_stop;
	c_right <= not w_up_down and not w_stop;
	
	c_fsmReset <= btnR or btnU;
	c_clkReset <= btnL or btnU;
	
    c_passFloorZeroCheck <= not (sw(15) or sw(14) or sw(13) or sw(12));
    c_destFloorZeroCheck <= not (sw(3) or sw(2) or sw(1) or sw(0));	
	 
    with c_passFloorZeroCheck select
				c_passFloor_next <= "0001" when '1',
									sw(15 downto 12) when others;


    with c_destFloorZeroCheck select
				c_destFloor_next <= "0001" when '1',
									sw(3 downto 0) when others;	

	destReg : process(btnC, c_fsmReset)
    begin
      if (c_fsmReset = '1') then
        f_destFloor <= "0001"; -- Floor1
      elsif rising_edge(btnC) then
        f_destFloor <= c_destFloor_next;
      end if;
    end process;
	
	passReg : process(btnC, c_fsmReset)
    begin
      if (c_fsmReset = '1') then
        f_passFloor <= "0001"; -- Floor1
      elsif rising_edge(btnC) then
        f_passFloor <= c_passFloor_next;
      end if;
    end process;
	
	-- ground unused LEDs (which is all of them for Minimum functionality)
	led(9 downto 6) <= (others => '0');
	led(0) <= w_lights_R(2);
	led(1) <= w_lights_R(2);
	led(2) <= w_lights_R(1);
	led(3) <= w_lights_R(1);
	led(4) <= w_lights_R(0);
	led(5) <= w_lights_R(0);
	
    led(15) <= w_lights_L(2);
    led(14) <= w_lights_L(2);
    led(13) <= w_lights_L(1);
    led(12) <= w_lights_L(1);
    led(11) <= w_lights_L(0);
    led(10) <= w_lights_L(0);


end top_basys3_arch;
