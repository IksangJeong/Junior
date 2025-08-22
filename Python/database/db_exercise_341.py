"""
데이터베이스 연습 #341
SQLite CRUD 실습
"""
import sqlite3

def setup_341():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_341 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_341_0', 401.63, 'C'), ('item_341_1', 617.97, 'A'), ('item_341_2', 455.08, 'B'), ('item_341_3', 394.95, 'C'), ('item_341_4', 222.65, 'B'), ('item_341_5', 754.8, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_341 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_341()
    cursor = conn.cursor()

    print(f"DB 연습 #341")
    cursor.execute('SELECT * FROM items_341')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_341 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
