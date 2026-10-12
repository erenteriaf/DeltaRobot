"""Move selection for the robot's side.

The cell was built around Stockfish, so this uses it when a binary is on PATH
or in sim/stockfish/. Without one it falls back to a small negamax with
alpha-beta over python-chess, enough to play a reasonable game against a person
and to keep everything running with nothing installed but python-chess.
"""
import glob
import os
import shutil

import chess
import chess.engine

PIECE_VALUE = {chess.PAWN: 100, chess.KNIGHT: 320, chess.BISHOP: 330,
               chess.ROOK: 500, chess.QUEEN: 900, chess.KING: 0}


class Engine:
    def __init__(self, depth=3, think_time=0.5):
        self.depth, self.think_time, self.stockfish = depth, think_time, None
        local = glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'stockfish', 'stockfish*'))
        path = local[0] if local else shutil.which('stockfish')
        if path:
            try:
                self.stockfish = chess.engine.SimpleEngine.popen_uci(path)
            except Exception:
                pass
        self.name = 'stockfish' if self.stockfish else f'negamax d{depth}'

    def close(self):
        if self.stockfish:
            self.stockfish.quit()

    def play(self, board):
        """Best move for the side to move, or None if the game is over."""
        if board.is_game_over():
            return None
        if self.stockfish:
            return self.stockfish.play(board, chess.engine.Limit(time=self.think_time)).move
        return self.negamax(board, self.depth, -10**9, 10**9)[1] or next(iter(board.legal_moves))

    def evaluate(self, board):
        """Material plus a little for standing in the middle, from the side to move."""
        if board.is_checkmate():
            return -10**6
        if board.is_stalemate() or board.is_insufficient_material():
            return 0
        score = 0
        for square, piece in board.piece_map().items():
            central = 7 - abs(3.5 - chess.square_file(square)) - abs(3.5 - chess.square_rank(square))
            value = PIECE_VALUE[piece.piece_type] + (0 if piece.piece_type == chess.KING else 2 * central)
            score += value if piece.color == board.turn else -value
        return score

    def negamax(self, board, depth, alpha, beta):
        if depth == 0 or board.is_game_over():
            return self.evaluate(board), None
        best = None
        for move in sorted(board.legal_moves, key=board.is_capture, reverse=True):   # Captures first, it cuts more
            board.push(move)
            score = -self.negamax(board, depth - 1, -beta, -alpha)[0]
            board.pop()
            if score > alpha:
                alpha, best = score, move
            if alpha >= beta:
                break
        return alpha, best
