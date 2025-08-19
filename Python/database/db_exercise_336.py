"""
데이터베이스 연습 #336
SQLite CRUD 실습
"""
import sqlite3

def setup_336():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_336 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_336_0', 504.31, 'C'), ('item_336_1', 309.8, 'B'), ('item_336_2', 662.08, 'C'), ('item_336_3', 428.9, 'B'), ('item_336_4', 83.23, 'C'), ('item_336_5', 484.44, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_336 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_336()
    cursor = conn.cursor()

    print(f"DB 연습 #336")
    cursor.execute('SELECT * FROM items_336')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_336 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
