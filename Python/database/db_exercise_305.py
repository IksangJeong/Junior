"""
데이터베이스 연습 #305
SQLite CRUD 실습
"""
import sqlite3

def setup_305():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_305 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_305_0', 539.8, 'C'), ('item_305_1', 851.83, 'A'), ('item_305_2', 914.1, 'A'), ('item_305_3', 423.71, 'A'), ('item_305_4', 934.58, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_305 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_305()
    cursor = conn.cursor()

    print(f"DB 연습 #305")
    cursor.execute('SELECT * FROM items_305')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_305 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
