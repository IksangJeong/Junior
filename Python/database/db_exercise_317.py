"""
데이터베이스 연습 #317
SQLite CRUD 실습
"""
import sqlite3

def setup_317():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_317 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_317_0', 70.35, 'C'), ('item_317_1', 880.92, 'A'), ('item_317_2', 629.73, 'B'), ('item_317_3', 560.29, 'A'), ('item_317_4', 972.57, 'C'), ('item_317_5', 674.63, 'C'), ('item_317_6', 325.55, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_317 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_317()
    cursor = conn.cursor()

    print(f"DB 연습 #317")
    cursor.execute('SELECT * FROM items_317')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_317 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
