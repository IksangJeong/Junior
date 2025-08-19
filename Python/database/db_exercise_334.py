"""
데이터베이스 연습 #334
SQLite CRUD 실습
"""
import sqlite3

def setup_334():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_334 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_334_0', 817.32, 'C'), ('item_334_1', 974.48, 'A'), ('item_334_2', 307.43, 'C'), ('item_334_3', 947.81, 'A'), ('item_334_4', 484.22, 'A'), ('item_334_5', 964.93, 'B'), ('item_334_6', 298.53, 'C'), ('item_334_7', 636.57, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_334 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_334()
    cursor = conn.cursor()

    print(f"DB 연습 #334")
    cursor.execute('SELECT * FROM items_334')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_334 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
