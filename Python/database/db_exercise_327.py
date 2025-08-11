"""
데이터베이스 연습 #327
SQLite CRUD 실습
"""
import sqlite3

def setup_327():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_327 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_327_0', 439.57, 'B'), ('item_327_1', 570.2, 'A'), ('item_327_2', 895.84, 'A'), ('item_327_3', 237.47, 'C'), ('item_327_4', 888.68, 'B'), ('item_327_5', 278.38, 'A'), ('item_327_6', 687.58, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_327 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_327()
    cursor = conn.cursor()

    print(f"DB 연습 #327")
    cursor.execute('SELECT * FROM items_327')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_327 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
