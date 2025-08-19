"""
데이터베이스 연습 #333
SQLite CRUD 실습
"""
import sqlite3

def setup_333():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_333 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_333_0', 175.54, 'B'), ('item_333_1', 426.89, 'B'), ('item_333_2', 784.84, 'B'), ('item_333_3', 259.18, 'A'), ('item_333_4', 723.69, 'B'), ('item_333_5', 597.87, 'A'), ('item_333_6', 463.28, 'A'), ('item_333_7', 311.61, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_333 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_333()
    cursor = conn.cursor()

    print(f"DB 연습 #333")
    cursor.execute('SELECT * FROM items_333')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_333 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
