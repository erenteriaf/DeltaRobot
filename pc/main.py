import os
import time
import pygame
import client_cognex
import plc_connection
from tictactoe import TicTacToe

pygame.mixer.init()

pygame.mixer.music.set_volume(0.5)  # Set volume to 50%
pygame.mixer.music.load(os.path.join(os.getcwd(), "sounds", "its_time.mp3"))
pygame.mixer.music.play()

board = [[" " for _ in range(3)] for _ in range(3)]
list_board = [cell for row in board for cell in row]

BOARD_COORIDNATES = {
    '0': {'x': -100.0, 'y': -15.0},
    '1': {'x': -25.0, 'y': 25.0},
    '2': {'x': 40.0, 'y': 70.0},
    '3': {'x': -140.0, 'y': 50.0},
    '4': {'x': -70.0, 'y': 100.0},
    '5': {'x': -5.0, 'y': 145.0},
    '6': {'x': -175.0, 'y': 120.0},
    '7': {'x': -105.0, 'y': 170.0},
    '8': {'x': -40.0, 'y': 220.0},
}

FREE_TOKEN_COORDINATES = {
    '0': {'x': 75.0, 'y': -15.0, 'z': -522.0},
    '1': {'x': -25.0, 'y': -75.0, 'z': -522.0},
    '2': {'x': -135.0, 'y': -135.0, 'z': -503.0},
    '3': {'x': -185.0, 'y': -45.0, 'z': -503.0},
    '4': {'x': -255.0, 'y': 30.0, 'z': -503.0},
    }

z_high = -400.0

z_low = - 500
#print("Cognex Data:", data)
print()

cheating = False  # Flag to indicate if cheating is enabled

token = 0 
game = TicTacToe(my_token='X', opponent_token='O', board=list_board)

previous_turn_state = 0  # Initialize previous state as off
waiting_for_off = False  # Flag to wait for signal to turn off

print("Starting Tic Tac Toe Game")

plc_connection.go_out_home()
time.sleep(4)
plc_connection.go_home()
time.sleep(4)

if game.game_over():
    print("Game already over, exiting.")
    
while not game.game_over():
    # send plc signal to go home
    #plc_connection.write_go_home_plc()  # Reset coordinates to home position
    #my_turn = plc_connection.read_my_turn()
    my_turn = plc_connection.read_my_turn_memory()  # Read the current turn state from PLC
    #my_turn = 1  # For testing purposes, we assume it's always our turn
    if my_turn == 1 and previous_turn_state == 0 and not waiting_for_off:
        print("It's my turn!")
        time.sleep(1)  # Wait for a moment to ensure the PLC signal is stable
        done_turn = False

        print('Token:', token)
        

        list_board = client_cognex.get_cognex_data()
        list_board.pop()  # Remove the last element which is not part of the board
        print('List Board:', list_board)

        if cheating:
            new_list_board = list_board.copy()
            for i in range(len(list_board)):
                if i == 'O':
                    new_list_board[i] = ' '
        
            game.board = new_list_board
        else:
            game.board = list_board
        print("Past Board State:")
        game.print_board()
        move = game.get_best_move()

        if move is None:
            print("Invalid move, exiting.")
            break
        
        
        game.make_move(move, game.my_token)

        print("Current Board State:")
        game.print_board()

        # Update PLC with the new board state
        board_x = BOARD_COORIDNATES[str(move)]['x']
        board_y = BOARD_COORIDNATES[str(move)]['y']

        token_x = FREE_TOKEN_COORDINATES[str(token)]['x']
        token_y = FREE_TOKEN_COORDINATES[str(token)]['y']

        print('Token:', token)
        
        print(f"Making move at position {move} with coordinates ({board_x}, {board_y})")

        waiting_for_off = True


        # If the board position is occupied by 'O', we need to throw the token
        if list_board[move] == 'O':
            # Go above the board position
            print(f"Moving to board position {move} at ({board_x}, {board_y})")
            print(f"Moving to Z high: {z_high}")
            plc_connection.write_oef(board_x, board_y, z_high)
            plc_connection.start_play()
            time.sleep(4)

            # Go down the board position
            print(f"Moving to Z low: {z_low}")
            plc_connection.write_oef(board_x, board_y, z_low-20)
            plc_connection.start_play()
            time.sleep(2)

            print("Closing gripper")
            plc_connection.close_gripper()
            time.sleep(1)

            # Go above the board position
            print(f"Moving to Z high: {z_high}")
            plc_connection.write_oef(board_x, board_y, z_high)
            plc_connection.start_play()
            time.sleep(2)

            # Throw the token
            print(f"Throwing token {token} at ({200}, {200})")
            plc_connection.write_oef(200, 200, z_high)
            plc_connection.start_play()
            time.sleep(2)

            # Open the gripper
            print("Open gripper")
            plc_connection.open_gripper()
            time.sleep(1)

        print(f"Moving to token {token} at ({token_x}, {token_y})")
        print(f"Moving to Z high: {z_high}")
        plc_connection.write_oef(token_x, token_y, z_high)
        plc_connection.start_play()
        time.sleep(4)
        # Go down
        print(f"Moving to Z low: {FREE_TOKEN_COORDINATES[str(token)]['z']}")
        plc_connection.write_oef(token_x, token_y, FREE_TOKEN_COORDINATES[str(token)]['z'])
        plc_connection.start_play()
        time.sleep(2)

        # Close the gripper
        print("Closing gripper")
        plc_connection.close_gripper()
        time.sleep(1)

    ## Go above the token
        print(f"Moving to Z high: {z_high}")
        plc_connection.write_oef(token_x, token_y, z_high)
        plc_connection.start_play()
        time.sleep(2)

        # Go above the board position
        print(f"Moving to board position {move} at ({board_x}, {board_y})")
        print(f"Moving to Z high: {z_high}")
        plc_connection.write_oef(board_x, board_y, z_high)
        plc_connection.start_play()
        time.sleep(4)

        # Go down the board position
        print(f"Moving to Z low: {z_low}")
        plc_connection.write_oef(board_x, board_y, z_low)
        plc_connection.start_play()
        time.sleep(2)

        # Open the gripper
        print("Open gripper")
        plc_connection.open_gripper()
        time.sleep(1)

        # Go above the board position
        print(f"Moving to board position {move} at ({board_x}, {board_y})")
        print(f"Moving to Z high: {z_high}")
        plc_connection.write_oef(board_x, board_y, z_high)
        plc_connection.start_play()
        time.sleep(2)

        # Go home
        print("Going home")
        plc_connection.activate_conveyor()
        plc_connection.go_home()
        time.sleep(10)
        plc_connection.deactivate_conveyor()
        token += 1
        if token > 4:
            token = 0
    elif my_turn == 0 and waiting_for_off:
        # Signal turned off, we can reset and wait for next rising edge
        waiting_for_off = False
        print("Signal turned off, waiting for next turn")
    
    # Update previous state
    previous_turn_state = my_turn
    

print("Game Over!")


    


