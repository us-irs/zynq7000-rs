----------------------------------------------------------------------------------
-- Company: Institute of Space Systems, University of Stuttgart
-- Engineer: Robin Mueller
--
-- Description:
-- The module multiplexes three UART modules.
-- It can be used to select between UART0 through EMIO, UARTLITE or UART16550.
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity uart_mux is
port(
  sys_clk : in std_logic;

  uart_0_tx : in std_logic;
  uart_0_rx : out std_logic;

  uart_1_tx : in std_logic;
  uart_1_rx : out std_logic;

  uart_2_tx : in std_logic;
  uart_2_rx : out std_logic;

  tx_out: out std_logic;
  rx_in: in std_logic;

  -- "000" -> UART0
  -- "001" -> UART1
  -- "010" -> UART2
  -- "011" -> UART0 to UART1
  -- "100" -> UART0 to UART2
  -- "101" -> UART1 to UART2
  sel : in std_logic_vector(2 downto 0)

	);
end uart_mux;

architecture Behavioral of uart_mux is

begin

switch : process(sys_clk)
begin
	if rising_edge(sys_clk) then
    case sel is
      when "000" =>
        tx_out <= uart_0_tx;
        uart_0_rx <= rx_in;
        uart_1_rx <= '1';
        uart_2_rx <= '1';
      when "001" =>
        tx_out <= uart_1_tx;
        uart_1_rx <= rx_in;
        uart_2_rx <= '1';
        uart_0_rx <= '1';
      when "010" =>
        tx_out <= uart_2_tx;
        uart_2_rx <= rx_in;
        uart_1_rx <= '1';
        uart_0_rx <= '1';
      when "011" =>
        tx_out <= '1';
        uart_1_rx <= uart_0_tx;
        uart_0_rx <= uart_1_tx;
        uart_2_rx <= '1';
      when "100" =>
        tx_out <= '1';
        uart_2_rx <= uart_0_tx;
        uart_0_rx <= uart_2_tx;
        uart_1_rx <= '1';
      when "101" =>
        tx_out <= '1';
        uart_1_rx <= uart_2_tx;
        uart_2_rx <= uart_1_tx;
        uart_0_rx <= '1';
      when others =>
        tx_out <= '1';
        uart_0_rx <= '1';
        uart_1_rx <= '1';
        uart_2_rx <= '1';
    end case;
	end if;
end process; -- switch

end Behavioral;
