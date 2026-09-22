"""Move selection for the robot's side.

The cell was built around Stockfish, so this uses it when a binary is on PATH
or in sim/stockfish/. When it is not, it falls back to a small negamax with
alpha-beta over python-chess move generation, which is enough to play a
reasonable game against a person and keeps the simulator runnable with nothing
installed but python-chess.
"""
import os
import shutil

import chess

PIECE_VALUE = {
    chess.PAWN: 100, chess.KNIGHT: 320, chess.BISHOP: 330,
    chess.ROOK: 500, chess.QUEEN: 900, chess.KING: 0,
}

# Encourages pieces off the back rank and towards the middle
CENTRALITY = [
    0,  5, 10, 15, 15, 10,  5,  0,
    5, 10, 20, 25, 25, 20, 10,  5,
    10, 20, 30, 35, 35, 30, 20, 10,
    15, 25, 35, 40, 40, 35, 25, 15,
    15, 25, 35, 40, 40, 35, 25, 15,
    10, 20, 30, 35, 35, 30, 20, 10,
    5, 10, 20, 25, 25, 20, 10,  5,
    0,  5, 10, 15, 15, 10,  5,  0,
]


def _find_stockfish():
    local = os.path.join(os.path.dirname(__file__), "stockfish")
    if os.path.isdir(local):
        for name in os.listdir(local):
            if name.lower().startswith("stockfish"):
                return os.path.join(local, name)
    return shutil.which("stockfish")


class Engine:
    def __init__(self, depth=3, think_time=0.5):
        self.depth = depth
        self.think_time = think_time
        self._sf = None
        path = _find_stockfish()
        if path:
            try:
                import chess.engine
                self._sf = chess.engine.SimpleEngine.popen_uci(path)
            except Exception:
                self._sf = None

    @property
    def name(self):
        return "stockfish" if self._sf else f"negamax d{self.depth}"

    def close(self):
        if self._sf:
            self._sf.quit()
            self._sf = None

    def play(self, board):
        """Best move for the side to move, or None if the game is over."""
        if board.is_game_over():
            return None
        if self._sf:
            import chess.engine
            return self._sf.play(board, chess.engine.Limit(time=self.think_time)).move
        _, move = self._negamax(board, self.depth, -10**9, 10**9)
        return move or next(iter(board.legal_moves), None)

    # ---------- fallback search ----------
    def _evaluate(self, board):
        if board.is_checkmate():
            return -10**6
        if board.is_stalemate() or board.is_insufficient_material():
            return 0
        score = 0
        for square, piece in board.piece_map().items():
            value = PIECE_VALUE[piece.piece_type]
            place = CENTRALITY[square] // 4 if piece.piece_type != chess.KING else 0
            total = value + place
            score += total if piece.color == board.turn else -total
        return score

    def _negamax(self, board, depth, alpha, beta):
        if depth == 0 or board.is_game_over():
            return self._evaluate(board), None
        best_move = None
        # captures first, it makes alpha-beta cut far more
        moves = sorted(board.legal_moves, key=board.is_capture, reverse=True)
        for move in moves:
            board.push(move)
            score, _ = self._negamax(board, depth - 1, -beta, -alpha)
            score = -score
            board.pop()
            if score > alpha:
                alpha, best_move = score, move
            if alpha >= beta:
                break
        return alpha, best_move
